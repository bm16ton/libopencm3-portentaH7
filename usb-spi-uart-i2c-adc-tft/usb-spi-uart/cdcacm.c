/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdlib.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/fsmc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>
#include <sys/types.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "cdc.h"
#include "dcache.h"
#include "usb.h"
#include "bulkspi.h"
#include "usb-2-i2c/i2c_ctx.h"
#include <libopencm3/stm32/syscfg.h>
#include "vars.h"
#include "tft_stm32_spi.h"
#include "fonts/pic.h"
//#include "fonts/16ton.h"
#include "fonts/bitmap_typedefs.h"
#include <libopencm3/cm3/mpu.h>
#include "mpustuff.h"
#include "debugen.h"
#include "adctest.h"
#include "locexti.h"
#include "xpt2046.h"
#include "fonts/font_ubuntu_48.h"
#include "ILI9486_Defines.h"
#include "fonts/font_fixedsys_mono_24.h"
#include "fonts/font_fixedsys_mono_16.h"
#include "fonts/lcd-debug-black.h"
#include <setjmp.h>
#include "fonts/firmupdate.h"
#include "console/console.h"

volatile uint16_t  menu1clr = ST_COLOR_YELLOW;
volatile uint16_t  menu2clr = ST_COLOR_YELLOW;
volatile int lcdcon = 0;
volatile int i2clcd = 0;
volatile int lcddma = 1;
#define USART_CONSOLE USART1
volatile uint32_t systick = 0;
volatile bool g_usbd_is_connected = false;
static usbd_device *g_usbd_dev = 0;
static void usb_puts(char *s);
static void usb_putc(unsigned int ign, char c);
void uart_dma_start(void *txbuf2, size_t data_size);
/*
#define DMA_BUFFER2 \
    __attribute__((aligned (2)))   __attribute__((section(".ccm")))

DMA_BUFFER2 __attribute__((aligned (2))) static uint8_t txbuf[1024];
*/

jmp_buf fatal_error_jmpbuf;
extern char _ebss[];
void aux_serial_get_encoding(usb_cdc_line_coding_s *const coding);

static uint32_t aux_serial_active_baud_rate;

void spiRelInit(void);
void spiInit(void);
void spi_test(void);
int i2cspeed = 2;
int bootnum = 0;
void get_buffered_line(void);
#define CDCACM_PACKET_SIZE 512
#define CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT_SIZE    64
static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xef,
	.bDeviceSubClass = 2,
	.bDeviceProtocol = 1,
	/* The size of the control endpoint for usb high speed devices *must* be 64, as dictated by the usb standard. */
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0403,
	.idProduct = 0xc631,
	.bcdDevice = 0x0205,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_endpoint_descriptor i2c_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x09,
};

const struct usb_interface_descriptor i2c_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 3,

	.endpoint = &i2c_endpoint,
};

static const struct usb_endpoint_descriptor endp_bulk2[] = {
    {
    	.bLength = USB_DT_ENDPOINT_SIZE,
	    .bDescriptorType = USB_DT_ENDPOINT,
	    .bEndpointAddress = 0x82,
	    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	    .wMaxPacketSize = 16,
	    .bInterval = 0x09,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x03,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = BULK_EP_MAXPACKET,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x83,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = BULK_EP_MAXPACKET,
		.bInterval = 1,
	},
};

static const struct usb_interface_descriptor iface_sourcesink2[] = {
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 1,
		.bAlternateSetting = 0,
		.bNumEndpoints = 3,
		.bInterfaceClass = USB_CLASS_VENDOR,
		.iInterface = 3,
		.endpoint = endp_bulk2,
	}
};

static const struct usb_iface_assoc_descriptor cdcacm_0_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = 2,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_AT,
	.iFunction = 2,
};


/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp_cdcacm_0[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x84,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 64,
	.bInterval = 16,
} };


static const struct usb_endpoint_descriptor data_endp_cdcacm_0[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x05,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 16,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x85,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 0,
} };


static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 3,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 2,
		.bSubordinateInterface0 = 3,
	 }
};

static const struct usb_interface_descriptor comm_iface_cdcacm_0[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 2,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 2,

	.endpoint = comm_endp_cdcacm_0,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
} };


static const struct usb_interface_descriptor data_iface_cdcacm_0[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 3,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 2,

	.endpoint = data_endp_cdcacm_0,
} };

static const struct usb_endpoint_descriptor endp_bulk1[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x06,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = BULK_EP_MAXPACKET,
		.bInterval = 0,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x86,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = BULK_EP_MAXPACKET,
		.bInterval = 0,
	},

};

static const struct usb_interface_descriptor iface_sourcesink1[] = {
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 4,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_VENDOR,
		.iInterface = 6,
		.endpoint = endp_bulk1,
	}
};


static const struct usb_interface ifaces[] = {
	{
		.num_altsetting = 1,
	    .altsetting = &i2c_iface,
	},
//	{
//		.num_altsetting = 1,
//		.iface_assoc = &gpio_assoc,
//	    .altsetting = &gpio_iface,
//	},
	{
		.num_altsetting = 1,
	    .altsetting = iface_sourcesink2,
	},
	{
		.num_altsetting = 1,
		.iface_assoc = &cdcacm_0_assoc,
		.altsetting = comm_iface_cdcacm_0,
	},
	{
		.num_altsetting = 1,
		.altsetting = data_iface_cdcacm_0,
	},
	{
		.num_altsetting = 1,
	    .altsetting = iface_sourcesink1,
	},
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 5,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0xC8,

	.interface = ifaces,
};

static const char * usb_strings[] = {
	"16ton productions",
	"Envie",
	"i2c-16ton",
	"github.com/bm16ton",
	"usb-2-spi",
	"usb-2-adc",
};

//void i2c_init(void);
uint32_t time_now(void);
uint64_t time64_now(void);

uint32_t SystemCoreClock = 480000;

static inline __attribute__((always_inline)) void __WFI(void)
{
	__asm volatile ("wfi");
}



/* commands from USB, must e.g. match command ids in kernel driver */
#define CMD_ECHO       0
#define CMD_GET_FUNC   1
#define CMD_SET_DELAY  2
#define CMD_GET_STATUS 3

#define CMD_I2C_IO     4
#define CMD_I2C_BEGIN  1  // flag fo I2C_IO
#define CMD_I2C_END    2  // flag fo I2C_IO

/* linux kernel flags */
#define I2C_M_TEN		0x10	/* we have a ten bit chip address */
#define I2C_M_RD		0x01
#define I2C_M_NOSTART		0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK		0x0800

/* To determine what functionality is present */
#define I2C_FUNC_I2C			0x00000001
#define I2C_FUNC_10BIT_ADDR		0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING	0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC	0x00000008 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_READ_WORD_DATA_PEC  0x00000800 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA_PEC 0x00001000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_PROC_CALL_PEC	0x00002000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL_PEC 0x00004000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL	0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK		0x00010000
#define I2C_FUNC_SMBUS_READ_BYTE	0x00020000
#define I2C_FUNC_SMBUS_WRITE_BYTE	0x00040000
#define I2C_FUNC_SMBUS_READ_BYTE_DATA	0x00080000
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA	0x00100000
#define I2C_FUNC_SMBUS_READ_WORD_DATA	0x00200000
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA	0x00400000
#define I2C_FUNC_SMBUS_PROC_CALL	0x00800000
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA	0x01000000
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK	0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK	0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2	 0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2 0x20000000 /* w/ 2-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA_PEC  0x40000000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC 0x80000000 /* SMBus 2.0 */

#define I2C_FUNC_SMBUS_BYTE I2C_FUNC_SMBUS_READ_BYTE | \
                            I2C_FUNC_SMBUS_WRITE_BYTE
#define I2C_FUNC_SMBUS_BYTE_DATA I2C_FUNC_SMBUS_READ_BYTE_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_BYTE_DATA
#define I2C_FUNC_SMBUS_WORD_DATA I2C_FUNC_SMBUS_READ_WORD_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_WORD_DATA
#define I2C_FUNC_SMBUS_BLOCK_DATA I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA
#define I2C_FUNC_SMBUS_I2C_BLOCK I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
                                  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK

#define I2C_FUNC_SMBUS_EMUL I2C_FUNC_SMBUS_QUICK | \
                            I2C_FUNC_SMBUS_BYTE | \
                            I2C_FUNC_SMBUS_BYTE_DATA | \
                            I2C_FUNC_SMBUS_WORD_DATA | \
                            I2C_FUNC_SMBUS_PROC_CALL | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC | \
                            I2C_FUNC_SMBUS_I2C_BLOCK

/* the currently support capability is quite limited */
const unsigned long func = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;


#define STATUS_IDLE	   0
#define STATUS_ADDRESS_ACK 1
#define STATUS_ADDRESS_NACK 2

static uint8_t status = STATUS_IDLE;

uint32_t i2c = I2C3;

/*!
 * \brief Handle I2C I/O request.
 *
 * \todo There is no bus error checking at all...
 */
static int usb_i2c_io(struct usb_setup_data *req, uint8_t *buf, uint16_t *len)
{
	uint32_t reg32 __attribute__((unused));

	/* Interpret the request */
	uint8_t cmd = req->bRequest;
	uint8_t address = req->wIndex;
	uint8_t is_read = req->wValue & I2C_M_RD;
	uint8_t size = req->wLength;

	i2c_ctx_t ctx;
    i2caddr = address;
	i2c_ctx_init(&ctx, I2C3);

	/* We can ignore CMD_I2C_BEGIN, the hardware will work out which
	 * type of start condition to generate.
	 */
	PT_CALL(&ctx.leaf, i2c_ctx_start(&ctx, address, size, is_read));
	if (ctx.err)
		goto err;

		PT_CALL(&ctx.leaf, is_read ? i2c_ctx_getdata(&ctx, buf, size)
					    : i2c_ctx_senddata(&ctx, buf, size));
    if (ctx.err) {
        goto err;
      }

	if (cmd & CMD_I2C_END && !is_read) {
		PT_CALL(&ctx.leaf, i2c_ctx_stop(&ctx));
		if (ctx.err)
			goto err;
	}

	status = STATUS_ADDRESS_ACK;
	*len = (is_read ? size : 0);
	return USBD_REQ_HANDLED;

err:
	i2c_ctx_reset(&ctx);
	status = STATUS_ADDRESS_NACK;
	*len = 0;
	return USBD_REQ_HANDLED;
}

static enum usbd_request_return_codes usb_control_request(
    usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	static uint8_t reply_buf[64];

	(void)usbd_dev;
	(void)complete;

	switch (req->bRequest) {
	case CMD_ECHO:
		memcpy(reply_buf, &req->wValue, sizeof(req->wValue));
		*buf = reply_buf;
		*len = sizeof(req->wValue);
		return USBD_REQ_HANDLED;

	case CMD_GET_FUNC:
		/* Report our capabilities */
		memcpy(reply_buf, &func, sizeof(func));
		*buf = reply_buf;
		*len = sizeof(func);
		return USBD_REQ_HANDLED;

	case CMD_SET_DELAY:
		/* This was used in i2c-tiny-usb to choose the clock
		 * frequency by specifying the shortest time between
		 * clock edges.
		 *
		 * This implementation silently ignores delay requests. We
		 * run the hardware as fast as we are permitted.
		 */
		*buf = reply_buf;
		*len = 0;
		return USBD_REQ_HANDLED;

	case CMD_I2C_IO:
	case CMD_I2C_IO | CMD_I2C_BEGIN:
	case CMD_I2C_IO | CMD_I2C_END:
	case CMD_I2C_IO | CMD_I2C_BEGIN | CMD_I2C_END:
		if (req->wValue & I2C_M_RD)
			*buf = reply_buf;
		return usb_i2c_io(req, *buf, len);
		break;

	case CMD_GET_STATUS:
		memcpy(reply_buf, &status, sizeof(status));
		*buf = reply_buf;
		*len = sizeof(status);
		return USBD_REQ_HANDLED;

	default:
		break;

	}

	return USBD_REQ_NEXT_CALLBACK;
}

void usb_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usb_control_request);
}


void i2c_init(void)
{
	rcc_periph_clock_enable(RCC_I2C3);

	i2c_reset(I2C3);
	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO7 | GPIO8);
	gpio_set_output_options(GPIOH, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO7 | GPIO8);
	gpio_set_af(GPIOH, GPIO_AF4, GPIO7 | GPIO8);
	i2c_peripheral_disable(I2C3);
	i2c_reset(I2C3);
    i2c_disable_analog_filter(I2C3);
	i2c_set_digital_filter(I2C3, 0);
    if (i2cspeed == 1) {
        i2c_set_speed(I2C3, i2c_speed_sm_100k, 8);
    } else if (i2cspeed == 2) {
        i2c_set_speed(I2C3, i2c_speed_fm_400k, 8);
    } else if (i2cspeed == 3) {
        i2c_set_speed(I2C3, i2c_speed_fmp_1m, 16);
    }
	i2c_set_7bit_addr_mode(I2C3);
	i2c_peripheral_enable(I2C3);
	i2c_set_own_7bit_slave_address(I2C3, 0x00);

	for (uint32_t loop = 0; loop < 1500; ++loop) {
        __asm__("nop");
    }
    if (bootnum == 0) {
	    i2c_ctx_t ctx;
	    i2c_ctx_init(&ctx, I2C3);
	    i2c_ctx_reset(&ctx);
    }
    bootnum = 1;
}

static uint64_t sys_tick_counter;

void sys_tick_handler(void)
{
  ++systick;
/*  if (sys_tick_counter % 1000000 == 0) {
  //	lcdshow();
    printf("time '%lld' \r\n", sys_tick_counter);
    printf("time_now 32 '%ld' \r\n", time_now());
    printf("time_now 64 '%lld' \r\n", time64_now());
	}
*/
	sys_tick_counter += 1000;  //10 seconds,

}

uint32_t time_now(void)
{
	return sys_tick_counter;
}

uint64_t time64_now(void)
{
	return sys_tick_counter;
}

void delay_ms( uint32_t ms ) {
  uint32_t next = systick + ms;
  while ( systick < next ) { __WFI(); }
}



uint32_t SysTick_Config(uint32_t ticks) {
(void)ticks;
systick_set_reload(480000-1);

  STK_CSR  = STK_CSR_CLKSOURCE |
                   STK_CSR_TICKINT   |
                   STK_CSR_ENABLE;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
}

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[256];
/*
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART1, '\r');
			}
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}
*/

int _write(int file, char *ptr, int len)
{
	uint64_t i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
	if (lcdcon == 1) {
	for (i = 0; i < (uint64_t)len; i++) {
	    console_putc(&consolebm, ptr[i]);
	    }
//	    st_draw_bitmap_nodma(385, 95, &LCDDEBUGBLACK);
	    return i;
	}
	else if (g_usbd_is_connected) {
		for (i = 0; i < (uint64_t)len; i++) {
			usb_putc(USART1, ptr[i]);
		}
		return i;
	    } else {
	    for (i = 0; i < (uint64_t)len; i++) {
		if (ptr[i] == '\n') {
			usart_send_blocking(USART1, '\r');
		}
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	  }
	}
	errno = EIO;
	return -1;
}

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding, usbd_device *usbd_dev);

static void serial_delay( void )
{
	for (unsigned i = 0; i < 30000; i++)
	  {
		__asm__("nop");
	  }
}

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding, usbd_device *usbd_dev)
{
(void)usbd_dev;

usart_set_baudrate(USART1, (coding->dwDTERate));
aux_serial_active_baud_rate = coding->dwDTERate;
printf("baud = %ld\r\n", coding->dwDTERate);

if (coding->dwDTERate == 38400) {
//    st_draw_bitmap_nodma(1, 110, &bm16ton);
    lcddma = 0;
//    gpio_set(GPIOC, GPIO6);
    gpio_clear(GPIOK, GPIO6);
    gpio_set(GPIOK, GPIO7);
    gpio_clear(GPIOK, GPIO5);
    printf("inside 38400 function\r\n");
    usbdebug();
//    gpio_clear(GPIOC, GPIO6);
//    st_draw_string(385, 95, "LCDDBG", menu1clr, &font_fixedsys_mono_24);
//	st_draw_string(385, 125, "LCDI2C", menu2clr, &font_fixedsys_mono_24);
st_draw_bitmap_nodma(360, 95, &LCDDEBUGBLACK);
    lcddma = 1;
} else if (coding->dwDTERate == 667) {
    printf("spi2 clock freq  %ld \r\n", rcc_get_spi_clk_freq(SPI2));
	printf("spi5 clock freq  %ld \r\n", rcc_get_spi_clk_freq(SPI5));
    printf("qspi clock freq  %ld \r\n", rcc_get_qspi_clk_freq(RCC_QSPI));
	printf("fmc clock freq  %ld \r\n", rcc_get_fmc_clk_freq(RCC_FMC));
	printf("usart1 clock = %ld \r\n", rcc_get_usart_clk_freq(USART1));
	printf("cpu clock = %ld \r\n", rcc_get_bus_clk_freq(RCC_CPUCLK));
	printf("RCC_PERCLK = %ld \r\n", rcc_get_bus_clk_freq(RCC_PERCLK));
	printf("sysclock = %ld \r\n", rcc_get_bus_clk_freq(RCC_SYSCLK));
	printf("m4 core clock = %ld \r\n", rcc_get_core2_clk_freq());
	printf("I2C3 clock = %ld \r\n", rcc_get_i2c_clk_freq(I2C3));
	printf("dmamux reqid %d \r\n", dmamux_get_dma_channel_request(DMAMUX1, 1));
//	tftmenu();
} else if (coding->dwDTERate == 666) {
    nvic_disable_irq(NVIC_OTG_HS_IRQ);
    exti_disable_request(EXTI1);
    nvic_disable_irq(NVIC_EXTI1_IRQ);
    dma_disable(DMA1, DMA_CHANNEL1);
    dma_stream_reset(DMA1, DMA_STREAM1);
    spi_disable_tx_dma(SPI5);
    spi_disable(SPI2);
    spi_disable(SPI5);
    scb_reset_system();
    scb_reset_core();
} else if (coding->dwDTERate == 1200) {
    lcdcon = 0;
    _st_write_command_16bit(ST7789_MADCTL);
     _st_write_command_16bit(0x36);
     _st_write_data_16bit(0x28);
     _st_write_data_16bit(0x00);
    printf("entered reboot function \r\n");
    rcc_periph_clock_enable(RCC_RTCAPB);
    printf("after rcc_periph_clock_enable  \r\n");

    nvic_disable_irq(NVIC_OTG_HS_IRQ);
    exti_disable_request(EXTI1);
    nvic_disable_irq(NVIC_EXTI1_IRQ);
    st_fill_screen_nodma(0xF0C3);
    st_draw_bitmap_nodma(40, 22, &firmupdate);

    spi_disable(SPI2);
    spi_disable(SPI5);

    pwr_disable_backup_domain_write_protect();
    printf("after pwr_cr1  \r\n");
    rtc_unlock();
    printf("after RTC_WPR  \r\n");
    rtc_set_init_flag();
    printf("after RTC_ISR  \r\n");
    serial_delay();
    printf("done waiting for rtc init, now setting magicboot\r\n");  //todo used as delay cuz its almost perfect
    __asm( "NOP" );
    __asm( "NOP" );
    __asm( "NOP" );
    __asm( "NOP" );
RTC_BKPXR(0) = 0xDF59;  //arduino also has DFU_MAGIC_SERIAL_ONLY_RESET   0xb0  curious not in mcuboot code that i could see
    __asm( "NOP" );
    __asm( "NOP" );
    __asm( "NOP" );
if (RTC_BKPXR(0) == 0xDF59) {
    gpio_toggle(GPIOK, GPIO6);
    //scb_reset_core();
    scb_reset_system();
    scb_reset_core();
} else {
    RTC_BKPXR(0) = 0xDF59;
    delay_ms(100);
    scb_reset_core();
//    scb_reset_system();
    printf("RTC_BKPXR not seen equal to 0xDF59 = %ld\n", RTC_BKPXR(0));
    }
} else {
lcddma = 0;
    st_draw_string(385, 95, "LCDDBG", menu1clr, &font_fixedsys_mono_24);
	st_draw_string(385, 125, "LCDI2C", menu2clr, &font_fixedsys_mono_24);
    lcddma = 1;
/*
if (coding->dwDTERate == 74880) {
debug();
}
*/
//todo add usb to uart

	usart_set_baudrate(USART1, coding->dwDTERate);
	aux_serial_active_baud_rate = coding->dwDTERate;
	if (coding->bParityType)
		usart_set_databits(USART1, (coding->bDataBits + 1 <= 8 ? 8 : 9));
	else
		usart_set_databits(USART1, (coding->bDataBits <= 8 ? 8 : 9));

	switch(coding->bCharFormat) {
	case 0:
		usart_set_stopbits(USART1, USART_STOPBITS_1);
		break;
	case 1:
		usart_set_stopbits(USART1, USART_STOPBITS_1_5);
		break;
	case 2:
	default:
		usart_set_stopbits(USART1, USART_STOPBITS_2);
		break;
	}

	switch(coding->bParityType) {
	case 0:
		usart_set_parity(USART1, USART_PARITY_NONE);
		break;
	case 1:
		usart_set_parity(USART1, USART_PARITY_ODD);
		break;
	case 2:
	default:
		usart_set_parity(USART1, USART_PARITY_EVEN);
		break;
	}

	}
}

void aux_serial_get_encoding(usb_cdc_line_coding_s *const coding)
{
	coding->dwDTERate = aux_serial_active_baud_rate;

	switch (usart_get_stopbits(USART1)) {
	case USART_STOPBITS_1:
		coding->bCharFormat = USB_CDC_1_STOP_BITS;
		break;

	case USART_STOPBITS_2:
	default:
		coding->bCharFormat = USB_CDC_2_STOP_BITS;
		break;
	}

	switch (usart_get_parity(USART1)) {
	case USART_PARITY_NONE:
	default:
		coding->bParityType = USB_CDC_NO_PARITY;
		break;
	case USART_PARITY_ODD:
		coding->bParityType = USB_CDC_ODD_PARITY;
		break;
	case USART_PARITY_EVEN:
		coding->bParityType = USB_CDC_EVEN_PARITY;
		break;
	}

	const uint32_t data_bits = usart_get_databits(USART1);
	if (coding->bParityType == USB_CDC_NO_PARITY)
		coding->bDataBits = data_bits;
	else
		coding->bDataBits = data_bits - 1;
}


// TODO REALY NEED THAT USB-2-UART BEN!

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	static uint8_t xbuf[7];
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		    g_usbd_is_connected = req->wValue & 1; /* Check RTS bit */
			return USBD_REQ_HANDLED;
		}
		case USB_CDC_REQ_SET_LINE_CODING:
		    if (*len < sizeof(struct usb_cdc_line_coding))
			    return USBD_REQ_NOTSUPP;
		    usbuart_set_line_coding((struct usb_cdc_line_coding *)*buf, usbd_dev);
		    return USBD_REQ_HANDLED;
		case USB_CDC_REQ_GET_LINE_CODING:
			if (*len < sizeof(usb_cdc_line_coding_s))
				return USBD_REQ_NOTSUPP;
			aux_serial_get_encoding((usb_cdc_line_coding_s *)*buf);
		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NEXT_CALLBACK;;
}

//static void usbuart_send_rx_packet(void);
//static uint8_t txbuf[1024]  __attribute__ ((aligned(2)));;  //Ben u fool doesnt memcpy remove the need for align?

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
int i = 0;
uint16_t len;
//uint8_t buf[CDCACM_PACKET_SIZE] = {0};
 uint8_t buf[1024] __attribute__ ((aligned(2)));
uint8_t txbuf[1024]  __attribute__ ((aligned(2)));;
//usbd_ep_nak_set(usbd_dev, ep, 1);
//redo:
usbd_ep_nak_set(usbd_dev, ep, 1);
__DSB();
//SCB_CleanDCache_by_Addr ((uint32_t *)0x24000000, (int32_t)524288);
	len = usbd_ep_read_packet(usbd_dev, ep, txbuf, CDCACM_PACKET_SIZE);
	if (len > 0) {
	SCB_CleanDCache_by_Addr ((uint32_t *)0x24000000, (int32_t)524288);
//	memcpy(txbuf, buf, len);
	uart_dma_start(txbuf, len);
	}
}

/*
static void cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
int i;
int len;
uint8_t buf[CDCACM_PACKET_SIZE] = {0};
usbuart_send_rx_packet();
	//len = usbd_ep_read_packet(usbd_dev, ep, buf, sizeof buf);
//	while (usbd_ep_write_packet(usbd_dev, ep, buf, len) == 0);
//printf("usb-tx-cb length = %d\r\n", len);
}
*/

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x05, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x85, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE, 0);
	usbd_ep_setup(usbd_dev, 0x84, USB_ENDPOINT_ATTR_INTERRUPT, CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT_SIZE, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}



static uint8_t buf_rx[576];
static uint8_t buf_rx2[576];
volatile uint16_t lastct = 0;
volatile uint16_t ct = 0;
volatile uint32_t poopsielst = 0;


static void usart_setup(void)
{
	/* Setup USART1 parameters. */
	nvic_enable_irq(NVIC_DMA1_STR2_IRQ);
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_oversample_16(USART1);
//	usart_enable_fifos(USART1);
//	memset(buf_rx, 0x0, 1024);

    rcc_periph_clock_enable(RCC_SYSCFG);
    rcc_periph_clock_enable(RCC_DMA1);
nvic_enable_irq(NVIC_USART1_IRQ);
    nvic_enable_irq(NVIC_DMA1_STR3_IRQ);
    nvic_set_priority(NVIC_USART1_IRQ, 16);
    nvic_set_priority(NVIC_DMA1_STR2_IRQ, 16);
//    nvic_set_priority(NVIC_DMA1_STR3_IRQ, 0);
    dma_stream_reset(DMA1, DMA_STREAM2);
    dma_set_priority_level(DMA1, DMA_STREAM2, DMA_SxCR_PL_LOW);
    dma_enable_direct_mode(DMA1, DMA_STREAM2);
    dma_disable_mburst(DMA1, DMA_STREAM2);
    dma_enable_pburst(DMA1, DMA_STREAM2);
    dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM2);
    dma_set_memory_size(DMA1, DMA_STREAM2, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM2, DMA_SxCR_PSIZE_8BIT);
    dma_set_as_flow_controller(DMA1, DMA_STREAM2);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM2);
	dma_set_transfer_mode(DMA1, DMA_STREAM2, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	dma_set_peripheral_address(DMA1, DMA_STREAM2, (uint32_t) &USART1_TDR);
	dma_set_memory_address(DMA1, DMA_STREAM2, (uint32_t)0x0000);
	dma_set_number_of_data(DMA1, DMA_STREAM2, (uint16_t)0x00);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM2);
	dma_set_fifo_threshold(DMA1, DMA_STREAM2, DMA_THRESHOLD_FULL);
	dma_enable_bufferable_transfers(DMA1, DMA_STREAM2);
//	dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM2);
//    usart_enable_tx_fifo_threshold_interrupt(USART1);
    dmamux_set_dma_channel_request(DMAMUX1, 2, 42);


// RX DMA
//	USART_CR1(USART1) |= USART_CR1_IDLEIE;
	usart_enable_idle_interrupt(USART1);
//    nvic_enable_irq(NVIC_DMA1_STR3_IRQ);
    nvic_set_priority(NVIC_DMA1_STR3_IRQ, 0);
    dma_stream_reset(DMA1, DMA_STREAM3);
    dma_enable_direct_mode(DMA1, DMA_STREAM3);
    dma_dblbuf_enable(DMA1, DMA_STREAM3);
    dma_set_priority_level(DMA1, DMA_STREAM3, DMA_SxCR_PL_VERY_HIGH);
    dma_set_memory_size(DMA1, DMA_STREAM3, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM3, DMA_SxCR_PSIZE_8BIT);
    dma_circ_enable(DMA1, DMA_STREAM3, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM3);
	dma_enable_peripheral_increment_mode(DMA1, DMA_STREAM3);
	dma_set_as_flow_controller(DMA1, DMA_STREAM3);
    dma_set_transfer_mode(DMA1, DMA_STREAM3, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA1, DMA_STREAM3, (uint32_t) & USART1_RDR);
    dma_set_memory_address(DMA1, DMA_STREAM3, (uint32_t) &buf_rx);
    dma_set_memory_address2(DMA1, DMA_STREAM3, (uint32_t) &buf_rx2);
    dma_set_number_of_data(DMA1, DMA_STREAM3, 504);
//	dma_set_fifo_threshold(DMA1, DMA_STREAM3, DMA_THRESHOLD_QUARTER);
//	dma_enable_bufferable_transfers(DMA1, DMA_STREAM3);
//	dma_half_transfer_interrupt_enable(DMA1, DMA_STREAM3);
dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM2);
dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM3);
	dmamux_set_dma_channel_request(DMAMUX1, 3, 41);
	dma_enable(DMA1, DMA_STREAM3);
    dma_enable_stream(DMA1, DMA_STREAM3);

	usart_enable_rx_dma(USART1);

	usart_enable(USART1);
}

void usart1_isr(void) {
USBUSART_ISR_TEMPLATE(USART1, NVIC_DMA1_STR3_IRQ);
}


void dma1_str3_isr(void) {
USBUSART_DMA_RX_ISR_TEMPLATE(NVIC_USART1_IRQ, DMA_STREAM3);
}

void uart_dma_start(void *txbuf2, size_t data_size) {
	dma_set_memory_address(DMA1, DMA_STREAM2, (uint32_t)txbuf2);
	dma_set_number_of_data(DMA1, DMA_STREAM2, data_size);
	__DSB();
	dma_enable(DMA1, DMA_CHANNEL2);
	dma_enable_stream(DMA1, DMA_STREAM2);
	usart_enable_tx_dma(USART1);
}

static void ulpi_pins(uint32_t gpioport, uint16_t gpiopins)
{
	gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, gpiopins);
	gpio_set_output_options(gpioport, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, gpiopins);
	gpio_set_af(gpioport, GPIO_AF10, gpiopins);
}

static void gpio_setup(void)
{

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

	/* Setup GPIO pins for USART1 rx. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

	/* Setup USART1 RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO10);

//    gpio_mode_setup(GPIOJ, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
//	gpio_set_output_options(GPIOJ, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO11);

//	gpio_set(GPIOJ, GPIO11);
//    gpio_clear(GPIOJ, GPIO11);


/* Setup GPIO pin GPIO5/6/7 on GPIO port K for LED. */
	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
	gpio_set_output_options(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO5);
	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_set_output_options(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO6);
	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO7);
	gpio_set_output_options(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7);
    gpio_set(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_clear(GPIOK, GPIO7);

	ulpi_pins(GPIOA, GPIO3 | GPIO5);
	ulpi_pins(GPIOB, GPIO0 | GPIO1 | GPIO5  | GPIO10 | GPIO11 | GPIO12 | GPIO13);
	ulpi_pins(GPIOC, GPIO0);
	ulpi_pins(GPIOH, GPIO4);
	ulpi_pins(GPIOI, GPIO11);
}

static void InitLdoAndPll(void) {
  // Clock configuration for this platform.
  // clang-format off
    gpio_mode_setup(GPIOH, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO1);
    gpio_set_output_options(GPIOH, GPIO_OTYPE_PP,
				GPIO_OSPEED_50MHZ, GPIO1);
	for (unsigned i = 0; i < 20; i++)
	  {
		__asm__("nop");
	  }
	  gpio_set(GPIOH, GPIO1);
  static const struct rcc_pll_config pll_config = {
    .sysclock_source  = RCC_PLL,
    .pll_source       = RCC_PLLCKSELR_PLLSRC_HSE,
    .hse_frequency    = 25000000UL,
    .pll1 = {
        .divm = 5U,     // 5 pre-multiplier
        .divn = 192U,   // 192 x 5 = 960mhz
        .divp = 2U,     // PLL1P post-divider gives 480MHz
        .divq = 32U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 2U,
    },
    .pll2 = {
        .divm = 5U,     // 2MHz PLL1 base clock pre-multiplier, cancels post div2.
        .divn = 192U,   // 192 x 5 = 960mhz
        .divp = 4U,     // 960 / 4 =  240MHz
        .divq = 16U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 4U,    // MAY NEED TO BE 4 FOR SDRAM
    },
    .pll3 = {
        .divm = 5U,     // 2MHz PLL1 base clock pre-multiplier, cancels post div2.
        .divn = 160U,   // 5 x 160 = 800MHz
        .divp = 4U,     // 800 / 4 = 200
        .divq = 4U,    // post-divider 32 gives 6.25MHz, 16 12.5mhz, 8 25mhz, 4 50mhz.
        .divr = 48U, //.divr = 48U,  // 8.33mhz for i2c 400K 16.66MHZ FMP
    },
    // Set CPU to PLL1P output at 480MHz.
    .core_pre  = RCC_D1CFGR_D1CPRE_BYP,
    .hpre = RCC_D1CFGR_D1HPRE_DIV2,   // Constrain HCLK below 240MHz by dividing core clock by 2.
    .ppre1 = RCC_D2CFGR_D2PPRE_DIV2,  // Constrain APB1 below 120MHz by dividing HCLK3 by 2.
    .ppre2 = RCC_D2CFGR_D2PPRE_DIV2,  // Constrain APB2 below 120MHz by dividing HCLK3 by 2.
    .ppre3 = RCC_D1CFGR_D1PPRE_DIV2,  // Constrain APB3 below 120MHz by dividing HCLK3 by 2.
    .ppre4 = RCC_D3CFGR_D3PPRE_DIV2,  // Constrain APB4 below 120MHz by dividing HCLK3 by 2.
    .flash_waitstates = 4,
    .voltage_scale = PWR_VOS_SCALE_0, // Highest setting, should support 480MHz operation.
  };
  // clang-format on
rcc_clock_setup_pll(&pll_config);

//   Setup SPI buses to use the HSI clock @ 64MHz for SPI log2 based dividers.
//    rcc_set_spi123_clksel(RCC_D2CCIP1R_SPI123SEL_PLL2P);  // PERCLK is defaulted to HSI.
    rcc_set_i2c123_clksel(RCC_D2CCIP2R_I2C123SEL_PLL3R);
//    rcc_set_spi45_clksel(RCC_D2CCIP1R_SPI45SEL_APB4);
    rcc_set_spi45_clksel(RCC_D2CCIP1R_SPI45SEL_PLL2Q);
//    rcc_set_spi45_clksel(RCC_D2CCIP1R_SPI45SEL_PLL3Q);
    //  RCC_D2CCIP1R_SPI45SEL_HSI
    //  RCC_D2CCIP1R_SPI45SEL_HSE
//    rcc_set_spi123_clksel(RCC_D2CCIP1R_SPI123SEL_PLL1Q);
    rcc_set_qspi_clksel(RCC_D1CCIPR_QSPISEL_PLL2R);
//  rcc_set_spi123_clksel(RCC_D2CCIP1R_SPI123SEL_PLL2P);
    rcc_set_spi123_clksel(RCC_D2CCIP1R_SPI123SEL_PLL3P);
//  rcc_set_spi45_clksel(RCC_D2CCIP1R_SPI45SEL_HSI);
//  rcc_set_peripheral_clk_sel(RCC_BDCR, RCC_BDCR_RTCSEL_LSI);
  // Set CANFD to use PLL1Q set at 80MHz.
//  rcc_set_fdcan_clksel(RCC_D2CCIP1R_FDCANSEL_PLL1Q);

  rcc_periph_clock_enable(RCC_USB1OTGHSEN);
  rcc_periph_clock_enable(RCC_USB1OTGHSULPIEN);

}


static struct {
	uint32_t	gpio;
	uint16_t	pins;
} sdram_pins[5] = {
	{GPIOD, GPIO0 | GPIO1 | GPIO8 | GPIO9 | GPIO10 | GPIO14 | GPIO15},
	{GPIOE, GPIO0 | GPIO1 | GPIO7 | GPIO8 | GPIO9 | GPIO10 |
			GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15 },
	{GPIOF, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO11 |
			GPIO12 | GPIO13 | GPIO14 | GPIO15 },
	{GPIOG, GPIO0 | GPIO1 | GPIO2 | GPIO4 | GPIO5 | GPIO8 | GPIO15},
	{GPIOH, GPIO2 | GPIO5 | GPIO3}
};

static struct sdram_timing timing = {
	.trcd = 2,		/* RCD Delay */
	.trp = 2,		/* RP Delay */
	.twr = 2,		/* Write Recovery Time */
	.trc = 7,		/* Row Cycle Delay */
	.tras = 4,		/* Self Refresh Time */
	.txsr = 7,		/* Exit Self Refresh Time */
	.tmrd = 2,		/* Load to Active Delay */
};

void spiRelInit(){
	rcc_periph_clock_enable(RCC_SPI2);
	    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE,
        GPIO2
        |  GPIO3
    );

    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO2 | GPIO3);

	gpio_set_af(GPIOC, GPIO_AF5,
       GPIO2
        |  GPIO3
    );

	    gpio_mode_setup(GPIOI, GPIO_MODE_AF, GPIO_PUPD_NONE,
        GPIO0
        |  GPIO1
    );

	gpio_set_output_options(GPIOI, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO0 |  GPIO1);

    gpio_set_af(GPIOI, GPIO_AF5,
       GPIO0
        |  GPIO1
    );

    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO8);
    gpio_clear(GPIOA, GPIO8);

}

void spiInit(){

spi_reset(SPI2);
    gpio_set(GPIOA, GPIO8);
    gpio_clear(GPIOA, GPIO8);
	SPI_IFCR(SPI2)|=SPI_IER_MODFIE;


	spi_disable_crc(SPI2);
	spi_enable_software_slave_management(SPI2);

	spi_disable_ss_output(SPI2);
	spi_set_full_duplex_mode(SPI2);
	spi_set_transfer_size(SPI2, 0x0);
	spi_send_msb_first(SPI2);

	spi_set_nss_high(SPI2);

	spi_set_baudrate_prescaler(SPI2, SPI_CFG1_MBR_CLK_DIV_2);
	spi_set_data_size(SPI2,SPI_CFG1_DSIZE_8BIT);
		spi_set_master_mode(SPI2);

		SPI_CR2(SPI2) =0;
		SPI_CFG2(SPI2) |= SPI_CFG2_AFCNTR | SPI_CFG2_SSOE ;
	spi_enable(SPI2);
	SPI_CR1(SPI2) |= SPI_CR1_CSTART;
	gpio_clear(GPIOA, GPIO8);

}

void spi2_isr(void) {
printf("spi interrupt fired");
uint8_t temp = spi_read8(SPI2);
}

static void usb_puts(char *s) {
if (g_usbd_is_connected) {
while (usbd_ep_write_packet(g_usbd_dev, 0x85, s, strlen(s)) == 0);
    }
}


static char output_buffer[196 + sizeof(uint64_t)];
static uint32_t output_buffer_pos;

static void usb_putc(unsigned int ign, char c) {
    (void)ign;
    output_buffer[output_buffer_pos++] = c;
    if ((c == '\n') || (output_buffer_pos >= sizeof(output_buffer)-1)) {
        if ((c == '\n') || (output_buffer_pos < sizeof(output_buffer)-1))
        output_buffer[output_buffer_pos++] = '\r';
        output_buffer[output_buffer_pos] = '\0';
        usb_puts(output_buffer);
        output_buffer_pos = 0;
    }
}

//SDRAM CURRENTLY WORKS, THO I COULD NOT GET M4 TO WORK WITH IT
int sdramsetup( void ) {

	uint32_t cr_tmp, tr_tmp; /* control, timing registers */


   rcc_periph_clock_enable(RCC_FMC);

  SCB_CPACR  |=  ( 0xF << 20 );
  SysTick_Config( SystemCoreClock / 1000 );

	for (int i = 0; i < 5; i++) {
		gpio_mode_setup(sdram_pins[i].gpio, GPIO_MODE_AF,
				GPIO_PUPD_NONE, sdram_pins[i].pins);
		gpio_set_output_options(sdram_pins[i].gpio, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, sdram_pins[i].pins);
		gpio_set_af(sdram_pins[i].gpio, GPIO_AF12, sdram_pins[i].pins);
	}
  printf( "Configure FMC...\r\n" );

  // Enable the FMC peripheral.
  FSMC_BCR(0)   |=  ( FSMC_BCR_FMCEN );

	cr_tmp  = FMC_SDCR_RPIPE_1CLK;
	cr_tmp |= FMC_SDCR_SDCLK_2HCLK;
	cr_tmp |= FMC_SDCR_CAS_3CYC;
	cr_tmp |= FMC_SDCR_NB4;
	cr_tmp |= FMC_SDCR_MWID_16b;
	cr_tmp |= FMC_SDCR_NR_12;
	cr_tmp |= FMC_SDCR_NC_8;


	FMC_SDCR1 |= (cr_tmp & FMC_SDCR_DNC_MASK);
	FMC_SDCR2 = cr_tmp;

	tr_tmp = sdram_timing(&timing);
	FMC_SDTR1 |= (tr_tmp & FMC_SDTR_DNC_MASK);
	FMC_SDTR2 = tr_tmp;

  FMC_SDCMR     =  ( ( 1 << FMC_SDCMR_MODE_MASK ) |
                                FMC_SDCMR_CTB1 );

  for (unsigned i = 0; i < 20; i++)
	  {
		__asm__("nop");
	  }

  while( FMC_SDSR & 0x00000020 ) {};

  FMC_SDCMR     =  ( ( 2 << FMC_SDCMR_MODE_MASK ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_SDSR & 0x00000020 ) {};


  FMC_SDCMR     =  ( ( 1 << FMC_SDCMR_NRFS_SHIFT ) |
                                ( 3 << FMC_SDCMR_MODE_SHIFT ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_SDSR & 0x00000020 ) {};

  FMC_SDCMR     =  ( ( 0x020 << FMC_SDCMR_MRD_SHIFT ) |
                                ( 4 << FMC_SDCMR_MODE_SHIFT ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_SDSR & 0x00000020 ) {};

  FMC_SDRTR     |=  ( 916 << FMC_SDRTR_COUNT_SHIFT );

  // Make sure writes are enabled.
  FMC_SDCR1 &= ~( FMC_SDCR_WP_ENABLE );

  printf( "Done configuring FMC.\r\n" );
  return 0;
}

static void tp_irq_pin_init(void);

static void tp_irq_pin_init(void)
{
    rcc_periph_clock_enable(RCC_SYSCFG);

    rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_set(GPIOC, GPIO6);
	exti_select_source(EXTI6, GPIOC);
    exti_set_trigger(EXTI6, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI6);
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}

void exti9_5_isr(void)
{
    volatile int xrw;
    volatile int yrw;
    volatile int xraw;
    volatile int yraw;
    volatile int16_t thr;
//    volatile int zraw1;
//    volatile int zraw2;
    g_usbd_is_connected = 0;
    lcdcon = 0;
    volatile uint32_t *magic = (uint32_t *)_ebss;
    gpio_clear(GPIOC, GPIO7);
    TFT_CS_IDLE;
    nvic_disable_irq(NVIC_EXTI9_5_IRQ);
    xrw = ts_get_x();
    yrw = ts_get_y();
    thr = threshholdv();
    xraw = ts_get_x_raw();
    yraw = ts_get_y_raw();
    printf("threshhold =  %d\r\n", thr);
    printf("xraw =  %d\r\n", xraw);
    printf("yraw =  %d\r\n", yraw);
    printf("xrw =  %d\r\n", xrw);
    printf("yrw =  %d\r\n", yrw);

    if ((xraw >= 600 && xraw <= 720 ) && (yraw >= 250 && yraw <= 450)) {
    TS_CS_IDLE;
    TFT_CS_ACTIVE;
    lcdcon = 1;
    i2clcd = 0;
    menu1clr = ST_COLOR_PURPLE;
    menu2clr = ST_COLOR_YELLOW;
    lcddma = 0;

    console_setup();
     _st_write_command_16bit(ST7789_MADCTL);
     _st_write_command_16bit(0x36);
     _st_write_data_16bit(0x28);
     _st_write_data_16bit(0x00);
    st_fill_screen_nodma(ILI9486_BLACK);
//    st_fill_rect_nodma(385, 95, 80, 25, ILI9486_RED);
    st_draw_string(385, 95, "LCDDBG", menu1clr, &font_fixedsys_mono_24);
//    st_fill_rect_nodma(385, 125, 80, 25, ILI9486_RED);
	st_draw_string(385, 125, "LCDI2C", menu2clr, &font_fixedsys_mono_24);
	lcddma = 1;
    }
    if ((xraw >= 850 && xraw <= 980 ) && (yraw >= 350 && yraw <= 450)) {
    TS_CS_IDLE;
    TFT_CS_ACTIVE;
    lcdcon = 0;
    i2clcd = 1;
    menu1clr = ST_COLOR_YELLOW;
    menu2clr = ST_COLOR_PURPLE;
    lcddma = 0;
    printf("test \r\n");
//     _st_write_command_16bit(ST7789_MADCTL);
//     _st_write_command_16bit(0x36);
//     _st_write_data_16bit(0x28);
//     _st_write_data_16bit(0x00);
    st_fill_screen_nodma(ILI9486_BLACK);
    printf("test \r\n");
//    st_fill_rect_nodma(385, 95, 80, 25, ILI9486_RED);
    st_draw_string(385, 95, "LCDDBG", menu1clr, &font_fixedsys_mono_24);
//    st_fill_rect_nodma(385, 125, 80, 25, ILI9486_RED);
//	st_draw_string(385, 125, "LCDI2C", menu2clr, &font_fixedsys_mono_24);
	lcddma = 1;
//adclive();
    }

    if ((xraw >= 1000 && xraw <= 1200 ) && (yraw >= 400 && yraw <= 600)) {
/*           magic[0] = BOOTMAGIC2;
           magic[1] = BOOTMAGIC3;
           printf("magic 0 =  0x%02lX\r\n", magic[0]);
           printf("magic 1 =  0x%02lX\r\n", magic[1]);
//	        GPIOA_MODER |= (0x00000000);
//bootjump();
            scb_reset_system(); // system for regular reset?
            scb_reset_core();  // core for entering bootloader?
*/

     }

    TS_CS_IDLE;
    TFT_CS_ACTIVE;

    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
__DSB();
if(exti_get_flag_status(EXTI6)) {
	exti_reset_request(EXTI6);
__DSB();
} else {
	exti_reset_request(EXTI6); // what why?
__DSB();
	}
    exti_reset_request(EXTI6); // oh good fucking god, why lord!?
__DSB();
}

void tftmenu(void) {
   if (lcdcon == 1) {
//     _st_write_command_16bit(0x36);
//    _st_write_data_16bit(0x28);
//    _st_write_data_16bit(0x00);
//     st_fill_screen(ST_COLOR_YELLOW);
     st_draw_rectangle(385, 95, 80, 25, ILI9486_RED);
	st_fill_rect(385, 95, 80, 25, ILI9486_RED);
	st_draw_string(385, 95, "LCDDBG", menu1clr, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 125, 80, 25, ILI9486_RED);
	st_fill_rect(385, 125, 80, 25, ILI9486_RED);
	st_draw_string(385, 125, "LCDI2C", menu2clr, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 155, 80, 25, ILI9486_RED);
	st_fill_rect(385, 155, 80, 25, ILI9486_RED);
	st_draw_string(385, 155, "LCDADC", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
  }
}

/*
void
local_heap_setup(uint8_t **start, uint8_t **end)
{
    *start = (uint8_t *)(0x60000000);
    *end = (uint8_t *)(0x60000000 + (8 * 1024 * 1024));
}
*/


void enable_mpu(void) ;
#define MPU_RASR_TEX_Pos                   19U                                            /*!< MPU RASR: ATTRS.TEX Position */
#define MPU_RASR_TEX_Msk                   (0x7UL << MPU_RASR_TEX_Pos)                    /*!< MPU RASR: ATTRS.TEX Mask */
#define ARM_MPU_REGION_SIZE_8MB      ((uint8_t)0x16U) ///!< MPU Region Size 8 MBytes
#define ARM_MPU_REGION_SIZE_512KB    ((uint8_t)0x12U) ///!< MPU Region Size 512 KBytes
#define ARM_MPU_REGION_SIZE_32KB     ((uint8_t)0x0EU) ///!< MPU Region Size 32 KBytes
//#define INNER_OUTER_NORMAL_NOCACHE_TYPE ((0b001 << MPU_RASR_TEX_Pos ) | ( 0b0 << MPU_RASR_C_Pos ) | ( 0b0 << MPU_RASR_B_Pos ) | ( 0b0 << MPU_RASR_S_Pos))

void enable_mpu(void) {
// disable mpu
__DSB();
__ISB();
MPU_CTRL = 0;
__DSB();
__ISB();

MPU_RNR = (0xFF << 3);
__DSB();
__ISB();
MPU_RBAR = ((0x40040000) | (1<<4) | (0xF << 3));
__DSB();
__ISB();
MPU_RASR = ((MPU_RASR_ENABLE) | (0x1F << MPU_RGNSZ_256KB) | (0 << 18) | (0 << 17) | (0 << 16) | (MPU_RASR_ATTR_AP_PRW_URW));
__DSB();
__ISB();

MPU_RNR = (0xFF << 4);
__DSB();
__ISB();
   // Configure region 0 to cover 512KB Flash (Normal, Non-Shared, Executable, Read-only)
   MPU_RBAR_A1 = ((0x60000000) | (1<<4) | (0xF << 4));
__DSB();
__ISB();
   MPU_RASR_A1 = ((MPU_RASR_ENABLE)| (0b001 << MPU_RASR_TEX_Pos ) | (0x1F << ARM_MPU_REGION_SIZE_8MB) | (MPU_RASR_ATTR_XN) |(0 << 18) | (0 << 17) | (0 << 16) | (MPU_RASR_ATTR_AP_PRW_URW));
   // Enable MPU

__DSB();
__ISB();
MPU_RNR = (0xFF << 2);
__DSB();
__ISB();
MPU_RBAR_A2 = ((0x30020000) | (1<<4) | (0xF << 2));
__DSB();
__ISB();
MPU_RASR_A2 = ((MPU_RASR_ENABLE) | (MPU_RASR_ATTR_B) | (0x1F << ARM_MPU_REGION_SIZE_32KB) | (0 << 18) | (0 << 17) | (0 << 16) | (MPU_RASR_ATTR_AP_PRW_URW));
__DSB();
__ISB();

MPU_RNR = (0xFF << 2);
__DSB();
__ISB();
MPU_RBAR_A3 = ((0x24000000) | (1<<4) | (0xF << 2));
__DSB();
__ISB();
MPU_RASR_A3 = ((MPU_RASR_ENABLE) | (MPU_RASR_ATTR_B) | (0x1F << ARM_MPU_REGION_SIZE_512KB) | (0 << 18) | (0 << 17) | (0 << 16) | (MPU_RASR_ATTR_AP_PRW_URW));
__DSB();
__ISB();
MPU_CTRL = ((MPU_CTRL_PRIVDEFENA) | (MPU_CTRL_ENABLE));
__DSB();
__ISB();
}


volatile uint32_t busy_count;
int main(void)
{
	SCB_VTOR = (uint32_t) 0x08040000;
//	volatile uint32_t *magic = (uint32_t *)_ebss;
    rcc_periph_clock_enable(RCC_GPIOH);
    rcc_periph_clock_enable(RCC_SYSCFG);
    rcc_periph_clock_enable(RCC_HSEM);   //MAINLY For booting M4
    rcc_periph_clock_enable(RCC_SRAM3);
    rcc_periph_clock_enable(RCC_SRAM2);

//	volatile int i;
    InitLdoAndPll();
//    if ((magic[0] == BOOTMAGIC6) && (magic[1] == BOOTMAGIC7))
//	{
//    magic[0] = 0;
//	magic[1] = 0;
	    //FW_ADDR has to be aligned to 0x100
//    bootjump();
//    }

//    SCB_EnableICache();  //can configure and enable
//SCB_EnableDCache();
//	SCB_DisableDCache();
//	SCB_CleanDCache();

	/* Enable clocks for LED & USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_GPIOG);

	rcc_periph_clock_enable(RCC_GPIOI);
	rcc_periph_clock_enable(RCC_GPIOJ);
	rcc_periph_clock_enable(RCC_GPIOK);
	gpio_setup();
	spiRelInit();
    rcc_periph_clock_enable(RCC_USART1);
    usart_setup();

 //   SCB_DisableDCache();
    SCB_CleanInvalidateDCache();
	spiInit();

	st_init();
	spi_status(SPI5, "after stInit :");
	st_fill_screen(ST_COLOR_BLACK);
	__DSB();

	console_setup();
	lcdcon = 1;
    tp_irq_pin_init();


    sdramsetup();
    enable_mpu();
    setcm4bootadd0(0x08100000 >> 16);
//    bootboth();
//    forcem4boot();
    gpio_set(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_clear(GPIOK, GPIO7);

    printf("m4boot0addr = 0x%04x\r\n", getcm4bootadd0());
    i2c_init();

    spi_status(SPI5, "after sdram setup :");

	spi_status(SPI5, "after spiRelInit setuo :");

	for (unsigned i = 0; i < 3000000; i++)
	  {
		__asm__("nop");
	  }
//	adc_dma_setup();
 //   adc_start();

	g_usbd_dev = usbd_init(&stm32f207_usb_driver, &dev, &config,
		usb_strings, 6,
			usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(g_usbd_dev, usb_set_config);
    usbd_register_set_config_callback(g_usbd_dev, usbspi_set_config);
    usbd_register_set_config_callback(g_usbd_dev, cdcacm_set_config);
    usbd_register_set_config_callback(g_usbd_dev, usbadc_set_config);

	nvic_enable_irq(NVIC_OTG_HS_IRQ);
    nvic_set_priority(NVIC_OTG_HS_IRQ, 0);

    gpio_clear(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_set(GPIOK, GPIO7);
//	ramtest2();
	printplls();

	gpio_set(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_clear(GPIOK, GPIO7);
//	usbdebug();

__DSB();
 twaitm = (37);

    printf("spi2 clock freq  %ld \r\n", rcc_get_spi_clk_freq(SPI2));
	printf("spi5 clock freq  %ld \r\n", rcc_get_spi_clk_freq(SPI5));
    printf("qspi clock freq  %ld \r\n", rcc_get_qspi_clk_freq(RCC_QSPI));
	printf("fmc clock freq  %ld \r\n", rcc_get_fmc_clk_freq(RCC_FMC));
	printf("usart1 clock = %ld \r\n", rcc_get_usart_clk_freq(USART1));
	printf("cpu clock = %ld \r\n", rcc_get_bus_clk_freq(RCC_CPUCLK));
	printf("RCC_PERCLK = %ld \r\n", rcc_get_bus_clk_freq(RCC_PERCLK));
	printf("sysclock = %ld \r\n", rcc_get_bus_clk_freq(RCC_SYSCLK));
	printf("m4 core clock = %ld \r\n", rcc_get_core2_clk_freq());
	printf("I2C3 clock = %ld \r\n", rcc_get_i2c_clk_freq(I2C3));
	printf("dmamux reqid %d \r\n", dmamux_get_dma_channel_request(DMAMUX1, 1));
	twaitm = (32);
//	st_fill_screen(ST_COLOR_YELLOW);

//	ramtest2();
SCB_CleanDCache_by_Addr ((uint32_t *)0x24000000, (int32_t)524288);
	ramtest();
//    printf("magic 0 =  0x%02lX\r\n", magic[0]);
//    printf("magic 1 =  0x%02lX\r\n", magic[1]);

//	if ((magic[0] == BOOTMAGIC2) && (magic[1] == BOOTMAGIC3))
//	{
//   magic[0] = 0;
//	magic[1] = 0;
	    //FW_ADDR has to be aligned to 0x100
//    bootjump();
//st_fill_screen(ST_COLOR_BLACK);
//    adclive();
//    }
//    tftmenu();
    st_draw_bitmap_nodma(340, 95, &LCDDEBUGBLACK);
	while (1) {
//	planets();
//	if (liveadc == 1) {
//		usbd_poll(g_usbd_dev);
//	usart_recv(USART1);
//	}
//adc_always();
	}
}

void dma1_str2_isr(void) {
	nvic_disable_irq(NVIC_OTG_HS_IRQ);
	dma_clear_interrupt_flags(DMA1, DMA_STREAM2);
	usart_clear_interrupt_flags_all(USART1);
	dma_disable(DMA1, DMA_CHANNEL2);
	dma_disable_stream(DMA1, DMA_STREAM2);
	usbd_ep_nak_set(g_usbd_dev, 0x05, 0);
	nvic_enable_irq(NVIC_OTG_HS_IRQ);
}

void exti1_isr(void)
{
    printf("interrupt fired\r\n");
    uint8_t buft[4] = {3, 3, 3, 3};
	exti_reset_request(EXTI1);
    usbd_ep_write_packet(g_usbd_dev, 0x82, buft, 4);
    exti_set_trigger(EXTI1, irqtype);
    printf("end of exti isr\r\n");
}

void otg_hs_isr()
{
	usbd_poll(g_usbd_dev);
	usbd_poll(g_usbd_dev);
}

