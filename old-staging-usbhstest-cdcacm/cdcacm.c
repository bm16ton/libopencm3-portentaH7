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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/fsmc.h>
#include <libopencm3/cm3/systick.h>
//#include "stm32h7xx.h"
//#include <libopencm3/stm32/h7/usart2.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "cdc.h"

/* Define this to nonzero, to have only one cdcacm interaface (usb serial port) active. */
#define SINGLE_CDCACM		0
#define USART_CONSOLE USART1
volatile uint32_t systick = 0;


void get_buffered_line(void);

enum
{
	/* WARNING: at this time, data IN endpoint sizes *must* equal the corresponding data OUT endpoint sizes. */
	CDCACM_INTERFACE_0_DATA_IN_ENDPOINT			= 0x81,
	CDCACM_INTERFACE_0_DATA_IN_ENDPOINT_SIZE		= 512,
	CDCACM_INTERFACE_0_DATA_OUT_ENDPOINT			= 0x01,
	CDCACM_INTERFACE_0_DATA_OUT_ENDPOINT_SIZE		= CDCACM_INTERFACE_0_DATA_IN_ENDPOINT_SIZE,
	CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT		= 0x82,
	CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT_SIZE	= 16,

	CDCACM_INTERFACE_1_DATA_IN_ENDPOINT			= 0x83,
	CDCACM_INTERFACE_1_DATA_IN_ENDPOINT_SIZE		= 512,
	CDCACM_INTERFACE_1_DATA_OUT_ENDPOINT			= 0x03,
	CDCACM_INTERFACE_1_DATA_OUT_ENDPOINT_SIZE		= CDCACM_INTERFACE_1_DATA_IN_ENDPOINT_SIZE,
	CDCACM_INTERFACE_1_NOTIFICATION_IN_ENDPOINT		= 0x84,
	CDCACM_INTERFACE_1_NOTIFICATION_IN_ENDPOINT_SIZE	= 16,

	MAX_USB_PACKET_SIZE					= 512,
	CDCACM_INTERFACE_COUNT					= 2,
};

static struct
{
	int		out_epnum;
	int		in_epnum;
	uint8_t		buf[MAX_USB_PACKET_SIZE];
	int		max_packet_size;
	int		len;
}
incoming_usb_data[CDCACM_INTERFACE_COUNT] =
{
	[0] =
	{
		.out_epnum		= CDCACM_INTERFACE_0_DATA_OUT_ENDPOINT,
		.in_epnum		= CDCACM_INTERFACE_0_DATA_IN_ENDPOINT,
		.max_packet_size	= CDCACM_INTERFACE_0_DATA_IN_ENDPOINT_SIZE,
		.len			= 0,
	},
	[1] =
	{
		.out_epnum		= CDCACM_INTERFACE_1_DATA_OUT_ENDPOINT,
		.in_epnum		= CDCACM_INTERFACE_1_DATA_IN_ENDPOINT,
		.max_packet_size	= CDCACM_INTERFACE_1_DATA_IN_ENDPOINT_SIZE,
		.len			= 0,
	},
};
static unsigned avaiable_incoming_endpoints;


static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xef,
	.bDeviceSubClass = 2,
	.bDeviceProtocol = 1,
	/* The size of the control endpoint for usb high speed devices *must* be 64, as dictated by the usb standard. */
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};


static const struct usb_iface_assoc_descriptor cdcacm_0_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = 0,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_AT,
	.iFunction = 0,
};

static const struct usb_iface_assoc_descriptor cdcacm_1_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = 2,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_AT,
	.iFunction = 0,
};


/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp_cdcacm_0[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT_SIZE,
	.bInterval = 11,
} };

static const struct usb_endpoint_descriptor comm_endp_cdcacm_1[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDCACM_INTERFACE_1_NOTIFICATION_IN_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = CDCACM_INTERFACE_1_NOTIFICATION_IN_ENDPOINT_SIZE,
	.bInterval = 11,
} };

static const struct usb_endpoint_descriptor data_endp_cdcacm_0[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDCACM_INTERFACE_0_DATA_OUT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_INTERFACE_0_DATA_OUT_ENDPOINT_SIZE,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDCACM_INTERFACE_0_DATA_IN_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_INTERFACE_0_DATA_IN_ENDPOINT_SIZE,
	.bInterval = 1,
} };

static const struct usb_endpoint_descriptor data_endp_cdcacm_1[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDCACM_INTERFACE_1_DATA_OUT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_INTERFACE_1_DATA_OUT_ENDPOINT_SIZE,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDCACM_INTERFACE_1_DATA_IN_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_INTERFACE_1_DATA_IN_ENDPOINT_SIZE,
	.bInterval = 1,
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
		.bDataInterface = 1,
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
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 }
};

static const struct usb_interface_descriptor comm_iface_cdcacm_0[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp_cdcacm_0,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
} };

static const struct usb_interface_descriptor comm_iface_cdcacm_1[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 2,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp_cdcacm_1,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
} };

static const struct usb_interface_descriptor data_iface_cdcacm_0[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp_cdcacm_0,
} };

static const struct usb_interface_descriptor data_iface_cdcacm_1[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 3,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp_cdcacm_1,
} };

static const struct usb_interface ifaces[] = {
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
		.iface_assoc = &cdcacm_1_assoc,
		.altsetting = comm_iface_cdcacm_1,
	},
	{
		.num_altsetting = 1,
		.altsetting = data_iface_cdcacm_1,
	},
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
#if SINGLE_CDCACM
	.bNumInterfaces = 2,
#else
	.bNumInterfaces = 4,
#endif
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char * usb_strings[] = {
	"Black Sphere Technologies",
	"CDC-ACM Demo",
	"DEMO",
};

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _siitcm, _sidtcm, _sitcm, _sdtcm, _eitcm, _edtcm, _estack;

uint32_t SystemCoreClock = 400000000;
void SysTick_IRQn_handler( void ) {
  ++systick;
}

void delay_ms( uint32_t ms ) {
  // Calculate the 'end of delay' tick value, then wait for it.
  uint32_t next = systick + ms;
  while ( systick < next ) { __WFI(); }
}

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[256];

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
								 /*
								  * This Linux cdc_acm driver requires this to be implemented
								  * even though it's optional in the CDC spec, and we don't
								  * advertise it in the ACM functional descriptor.
								  */
								 return USBD_REQ_HANDLED;
							 }
		case 0x21:
							 /* get line coding */
							 if (* len != 7)
								 return USBD_REQ_NOTSUPP;
							 memcpy(*buf, xbuf, 7);
							 return USBD_REQ_HANDLED;
		case 0x20:
							 memcpy(xbuf, *buf, 7);

							 return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
int i;
	/* Locate endpoint data. */
	for (i = 0; i < CDCACM_INTERFACE_COUNT; i ++)
		if (incoming_usb_data[i].out_epnum == ep)
			break;
	if (i == CDCACM_INTERFACE_COUNT)
		/* Endpoint not found. */
		return;

	incoming_usb_data[i].len = usbd_ep_read_packet(usbd_dev, ep, incoming_usb_data[i].buf, sizeof incoming_usb_data[i].buf);
	if (incoming_usb_data[i].len)
		usbd_ep_nak_set(usbd_dev, ep, 1);
	avaiable_incoming_endpoints |= 1 << i;
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, CDCACM_INTERFACE_0_DATA_OUT_ENDPOINT, USB_ENDPOINT_ATTR_BULK, CDCACM_INTERFACE_0_DATA_OUT_ENDPOINT_SIZE, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, CDCACM_INTERFACE_0_DATA_IN_ENDPOINT, USB_ENDPOINT_ATTR_BULK, CDCACM_INTERFACE_0_DATA_IN_ENDPOINT_SIZE, NULL);
	usbd_ep_setup(usbd_dev, CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT, USB_ENDPOINT_ATTR_INTERRUPT, CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT_SIZE, NULL);

	if (!SINGLE_CDCACM)
	{
		usbd_ep_setup(usbd_dev, CDCACM_INTERFACE_1_DATA_OUT_ENDPOINT, USB_ENDPOINT_ATTR_BULK, CDCACM_INTERFACE_1_DATA_OUT_ENDPOINT_SIZE, cdcacm_data_rx_cb);
		usbd_ep_setup(usbd_dev, CDCACM_INTERFACE_1_DATA_IN_ENDPOINT, USB_ENDPOINT_ATTR_BULK, CDCACM_INTERFACE_1_DATA_IN_ENDPOINT_SIZE, NULL);
		usbd_ep_setup(usbd_dev, CDCACM_INTERFACE_1_NOTIFICATION_IN_ENDPOINT, USB_ENDPOINT_ATTR_INTERRUPT, CDCACM_INTERFACE_1_NOTIFICATION_IN_ENDPOINT_SIZE, NULL);
	}

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

static void usart_setup(void)
{
	/* Setup USART1 parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void ulpi_pins(uint32_t gpioport, uint16_t gpiopins)
{
	gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, gpiopins);
	gpio_set_output_options(gpioport, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, gpiopins);
	gpio_set_af(gpioport, GPIO_AF10, gpiopins);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO5 on GPIO port K for LED. */
	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

	/* Setup GPIO pins for USART1 rx. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

	/* Setup USART1 RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO10);

	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
	gpio_set_output_options(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO5);
	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_set_output_options(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO6);
	gpio_mode_setup(GPIOK, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO7);
	gpio_set_output_options(GPIOK, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO7);
    gpio_set(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_clear(GPIOK, GPIO7);

    //ulpi

	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO3);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO3);

	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO5);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO0);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO1);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO1);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO5);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO10);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO10);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO11);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO11);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO12);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO12);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO13);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO13);

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO0);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0);

	gpio_mode_setup(GPIOH, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO4);
	gpio_set_output_options(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO4);

	gpio_mode_setup(GPIOI, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO11);
	gpio_set_output_options(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO11);

	gpio_set_af(GPIOA, GPIO_AF10, GPIO3);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO5);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO0);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO1);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO5);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO10);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO11);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO12);
	gpio_set_af(GPIOB, GPIO_AF10, GPIO13);
	gpio_set_af(GPIOC, GPIO_AF10, GPIO0);
	gpio_set_af(GPIOH, GPIO_AF10, GPIO4);
	gpio_set_af(GPIOI, GPIO_AF10, GPIO11);

	ulpi_pins(GPIOA, GPIO3 | GPIO5);
	ulpi_pins(GPIOB, GPIO0 | GPIO1 | GPIO5  | GPIO10 | GPIO11 | GPIO12 | GPIO13);
	ulpi_pins(GPIOC, GPIO0);
	ulpi_pins(GPIOH, GPIO4);
	ulpi_pins(GPIOI, GPIO11);

}

static void InitLdoAndPll(void) {
  // Clock configuration for this platform.
  // clang-format off
  static const struct rcc_pll_config pll_config = {
    .sysclock_source  = RCC_PLL,
    .pll_source       = RCC_PLLCKSELR_PLLSRC_HSE,
    .hse_frequency    = 25000000UL,
    .pll1 = {
        .divm = 5U,     // 2MHz PLL1 base clock pre-multiplier, cancels post div2.
        .divn = 160U,   // 800MHz for PLL1 to drive the sysclk throu post-div of 2.
        .divp = 2U,     // PLL1P post-divider gives 400MHz output.
        .divq = 16U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 2U,                // PLL1R are disabled for now.
    },
    .pll2 = {
        .divm = 5U,     // 2MHz PLL1 base clock pre-multiplier, cancels post div2.
        .divn = 80U,   // 800MHz for PLL1 to drive the sysclk throu post-div of 2.
        .divp = 2U,     // PLL1P post-divider gives 400MHz output.
        .divq = 2U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 2U,    // MAY NEED TO BE 4 FOR SDRAM          // PLL1R are disabled for now.
    },
    .pll3 = {
        .divm = 5U,     // 2MHz PLL1 base clock pre-multiplier, cancels post div2.
        .divn = 160U,   // 800MHz for PLL1 to drive the sysclk throu post-div of 2.
        .divp = 2U,     // PLL1P post-divider gives 400MHz output.
        .divq = 5U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 2U,                // PLL1R are disabled for now.
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

  // Setup SPI buses to use the HSI clock @ 64MHz for SPI log2 based dividers.
rcc_set_spi123_clksel(RCC_D2CCIP1R_SPI123SEL_PERCK);  // PERCLK is defaulted to HSI.
  rcc_set_spi45_clksel(RCC_D2CCIP1R_SPI45SEL_HSI);

  // Set CANFD to use PLL1Q set at 80MHz. 
  rcc_set_fdcan_clksel(RCC_D2CCIP1R_FDCANSEL_PLL1Q);
  
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


int sdramsetup( void ) {
	int i;
	uint32_t cr_tmp, tr_tmp; /* control, timing registers */

/*  RCC->AHB3ENR  |=  ( RCC_AHB3ENR_FMCEN );
  RCC->AHB4ENR  |=  ( RCC_AHB4ENR_GPIOAEN |
                      RCC_AHB4ENR_GPIOBEN |
                      RCC_AHB4ENR_GPIOCEN |
                      RCC_AHB4ENR_GPIODEN |
                      RCC_AHB4ENR_GPIOEEN |
                      RCC_AHB4ENR_GPIOFEN |
                      RCC_AHB4ENR_GPIOGEN );
  */ 
   rcc_periph_clock_enable(RCC_FMC);

   
//    memcpy( &_edata,  ( ( void* )&_edata ));
//  memcpy( &_sitcm, &_siitcm, ( ( void* )&_eitcm - ( void* )&_sitcm ) );
//  memcpy( &_sdtcm, &_sidtcm, ( ( void* )&_edtcm - ( void* )&_sdtcm ) );
  // Clear the .bss section in RAM.
//  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );
//  memcpy( &_edata, &_edata, sizeof(&_edata) );
//  memcpy( &_sitcm, &_siitcm, ( ( void* )&_eitcm - ( void* )&_sitcm ) );
//  memcpy( &_eccm, &_eccm, sizeof(_eccm) );
  // Clear the .bss section in RAM.
 // memset( &_ebss, 0x00, sizeof(&_ebss) );
  // Enable floating-point unit.
//  SCB->CPACR      |=  ( 0xF << 20 );
  SCB_CPACR  |=  ( 0xF << 20 );
  SysTick_Config( SystemCoreClock / 1000 );
    // Set SDRAM pins to high-speed alt. func. mode.
    /*
  GPIOA_MODER    &= ~( 3 << ( 7 * 2 ) );
  GPIOA_MODER    |=  ( 2 << ( 7 * 2 ) );
  GPIOA_OSPEEDR  |=  ( 3 << ( 7 * 2 ) );
  GPIOA_AFRL |=  ( 12 << ( 7 * 4 ) );
  GPIOC_MODER    &= ~( ( 3 << ( 4 * 2 ) ) | ( 3 << ( 5 * 2 ) ) );
  GPIOC_MODER    |=  ( ( 2 << ( 4 * 2 ) ) | ( 2 << ( 5 * 2 ) ) );
  GPIOC_OSPEEDR  |=  ( ( 3 << ( 4 * 2 ) ) | ( 3 << ( 5 * 2 ) ) );
  GPIOC_AFRL |=  ( ( 12 << ( 4 * 4 ) ) | ( 12 << ( 5 * 4 ) ) );
  GPIOD_MODER    &= ~( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 8 * 2 ) ) | ( 3 << ( 9 * 2 ) ) |
                        ( 3 << ( 10 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOD_MODER    |=  ( ( 2 << ( 0 * 2 ) ) | ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 8 * 2 ) ) | ( 2 << ( 9 * 2 ) ) |
                        ( 2 << ( 10 * 2 ) ) | ( 2 << ( 14 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOD_OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 8 * 2 ) ) | ( 3 << ( 9 * 2 ) ) |
                        ( 3 << ( 10 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOD_AFRL |=  ( ( 12 << ( 0 * 4 ) ) | ( 12 << ( 1 * 4 ) ) );
  GPIOD_AFRH |=  ( ( 12 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 9 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 10 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 14 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
  GPIOE_MODER    &= ~( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 7 * 2 ) ) | ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 9 * 2 ) ) | ( 3 << ( 10 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) | ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOE_MODER    |=  ( ( 2 << ( 0 * 2 ) ) | ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 7 * 2 ) ) | ( 2 << ( 8 * 2 ) ) |
                        ( 2 << ( 9 * 2 ) ) | ( 2 << ( 10 * 2 ) ) |
                        ( 2 << ( 11 * 2 ) ) | ( 2 << ( 12 * 2 ) ) |
                        ( 2 << ( 13 * 2 ) ) | ( 2 << ( 14 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOE_OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 7 * 2 ) ) | ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 9 * 2 ) ) | ( 3 << ( 10 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) | ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOE_AFRL |=  ( ( 12 << ( 0 * 4 ) ) | ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 7 * 4 ) ) );
  GPIOE_AFRH |=  ( ( 12 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 9 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 10 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 11 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 12 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 13 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 14 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
  GPIOF_MODER    &= ~( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) | ( 3 << ( 3 * 2 ) ) |
                        ( 3 << ( 4 * 2 ) ) | ( 3 << ( 5 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) | ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOF_MODER    |=  ( ( 2 << ( 0 * 2 ) ) | ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 2 * 2 ) ) | ( 2 << ( 3 * 2 ) ) |
                        ( 2 << ( 4 * 2 ) ) | ( 2 << ( 5 * 2 ) ) |
                        ( 2 << ( 11 * 2 ) ) | ( 2 << ( 12 * 2 ) ) |
                        ( 2 << ( 13 * 2 ) ) | ( 2 << ( 14 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOF_OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) | ( 3 << ( 3 * 2 ) ) |
                        ( 3 << ( 4 * 2 ) ) | ( 3 << ( 5 * 2 ) ) |
                        ( 3 << ( 11 * 2 ) ) | ( 3 << ( 12 * 2 ) ) |
                        ( 3 << ( 13 * 2 ) ) | ( 3 << ( 14 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOF_AFRL |=  ( ( 12 << ( 0 * 4 ) ) | ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 2 * 4 ) ) | ( 12 << ( 3 * 4 ) ) |
                        ( 12 << ( 4 * 4 ) ) | ( 12 << ( 5 * 4 ) ) );
  GPIOF_AFRH |=  ( ( 12 << ( ( 11 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 12 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 13 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 14 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
  GPIOG_MODER    &= ~( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) | ( 3 << ( 4 * 2 ) ) |
                        ( 3 << ( 5 * 2 ) ) | ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOG_MODER    |=  ( ( 2 << ( 0 * 2 ) ) | ( 2 << ( 1 * 2 ) ) |
                        ( 2 << ( 2 * 2 ) ) | ( 2 << ( 4 * 2 ) ) |
                        ( 2 << ( 5 * 2 ) ) | ( 2 << ( 8 * 2 ) ) |
                        ( 2 << ( 15 * 2 ) ) );
  GPIOG_OSPEEDR  |=  ( ( 3 << ( 0 * 2 ) ) | ( 3 << ( 1 * 2 ) ) |
                        ( 3 << ( 2 * 2 ) ) | ( 3 << ( 4 * 2 ) ) |
                        ( 3 << ( 5 * 2 ) ) | ( 3 << ( 8 * 2 ) ) |
                        ( 3 << ( 15 * 2 ) ) );
  GPIOG_AFRL |=  ( ( 12 << ( 0 * 4 ) ) | ( 12 << ( 1 * 4 ) ) |
                        ( 12 << ( 2 * 4 ) ) | ( 12 << ( 4 * 4 ) ) |
                        ( 12 << ( 5 * 4 ) ) );
  GPIOG_AFRH |=  ( ( 12 << ( ( 8 - 8 ) * 4 ) ) |
                        ( 12 << ( ( 15 - 8 ) * 4 ) ) );
*/
	for (i = 0; i < 5; i++) {
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

	/* We're programming BANK 2, but per the manual some of the parameters
	 * only work in CR1 and TR1 so we pull those off and put them in the
	 * right place.
	 */
	FMC_SDCR1 |= (cr_tmp & FMC_SDCR_DNC_MASK);
	FMC_SDCR2 = cr_tmp;

	tr_tmp = sdram_timing(&timing);
	FMC_SDTR1 |= (tr_tmp & FMC_SDTR_DNC_MASK);
	FMC_SDTR2 = tr_tmp;
  // Remap SDRAM1 to 0x60000000.
  /*
  FSMC_BANK1->BTCR[ 0 ]    |=  ( 1 << FMC_BCR1_BMAP_Pos );
    FSMC_BANK1->SDCR[ 0 ]  =  ( ( 1 << FMC_SDCRx_NC_Pos ) |
                                 ( 2 << FMC_SDCRx_NR_Pos ) |
                                 ( 1 << FMC_SDCRx_MWID_Pos ) |
                                 ( FMC_SDCRx_NB ) |
                                 ( 2 << FMC_SDCR_CAS_2CYC ) |
                                 ( 3 << FMC_SDCRx_SDCLK_Pos ) );

  FSMC_BANK1->SDTR[ 0 ]  =  ( ( 2 << FMC_SDTRx_TMRD_Pos ) |
                                 ( 8 << FMC_SDTRx_TXSR_Pos ) |
                                 ( 5 << FMC_SDTRx_TRAS_Pos ) |
                                 ( 8 << FMC_SDTRx_TRC_Pos ) |
                                 ( 2 << FMC_SDTRx_TWR_Pos ) |
                                 ( 2 << FMC_SDTRx_TRP_Pos ) |
                                 ( 2 << FMC_SDTRx_TRCD_Pos ) );
*/                                 
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

	      // Set 'MODE' bits to 3 and set # of consecutive auto-refreshes (2).
  /*
  //FSMC_BANK1->SDCMR     |=  ( 1 << FMC_SDCMR_NRFS_Pos );
  FSMC_BANK1->SDCMR     |=  ( 7 << FMC_SDCMR_NRFS_Pos );
  FSMC_BANK1->SDCMR     &= ~( FMC_SDCMR_MODE );
  FSMC_BANK1->SDCMR     |=  ( 3 << FMC_SDCMR_MODE_Pos );
  */
  FMC_SDCMR     =  ( ( 1 << FMC_SDCMR_NRFS_SHIFT ) |
                                ( 3 << FMC_SDCMR_MODE_SHIFT ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_SDSR & 0x00000020 ) {};

	   
  // Send 'Load Mode Register' command.
  //   * Burst length: 1
  //   * Burst type: Sequential
  //   * CAS latency: 2 cycles
  //   * Operating mode: Standard
  //   * Write burst: Same as read
  //FSMC_BANK1->SDCMR     |=  ( 0x020 << FMC_SDCMR_MRD_Pos );
  //FSMC_BANK1->SDCMR     &= ~( FMC_SDCMR_MODE );
  //FSMC_BANK1->SDCMR     |=  ( 4 << FMC_SDCMR_MODE_Pos );
  FMC_SDCMR     =  ( ( 0x020 << FMC_SDCMR_MRD_SHIFT ) |
                                ( 4 << FMC_SDCMR_MODE_SHIFT ) |
                                FMC_SDCMR_CTB1 );

  // Wait for the undocumented 'busy' bit to clear.
  while( FMC_SDSR & 0x00000020 ) {};
 
   // Configure SDRTR Refresh Timing Register:
  // 7.8us * 120MHz - 20 = 916.
  FMC_SDRTR     |=  ( 916 << FMC_SDRTR_COUNT_SHIFT );

  // Make sure writes are enabled.
  FMC_SDCR1 &= ~( FMC_SDCR_WP_ENABLE );

  printf( "Done.\r\n" );
  return 0;
}	
	
int ramtest(void) {
	  // Test external RAM reads and writes.
  uint32_t* sdram  = ( uint32_t* )0x60000000;
  uint16_t* sdramh = ( uint16_t* )0x60000000;
  uint8_t*  sdramb = ( uint8_t*  )0x60000000;
  printf( "RAM[0]: 0x%08lX (Uninitialized)\r\n", sdram[ 0 ] );
  sdram[ 0 ] = 0x01234567;
  sdram[ 1 ] = 0x89ABCDEF;
  printf( "RAM[0]: 0x%02X (Byte)\r\n", sdramb[ 0 ] );
  printf( "RAM[0]: 0x%04X (Halfword)\r\n", sdramh[ 0 ] );
  printf( "RAM[0]: 0x%08lX (Word)\r\n", sdram[ 0 ] );
  printf( "RAM[4]: 0x%08lX\r\n", sdram[ 1 ] );

  // Done; infinite loop.
 //  while( 1 ) {};
  return 0; // lol
}
	
	
	    
usbd_device *usbd_dev;
volatile uint32_t busy_count;
int main(void)
{
	SCB_VTOR = (uint32_t) 0x08040000;

	/* if this does not get incremented, it is possible that some usb interrupt flag is not handled,
	 * and the usb interrupt handler gets continuously invoked. */
	volatile int i;
    InitLdoAndPll();
	/* Enable clocks for LED & USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_GPIOG);
	rcc_periph_clock_enable(RCC_GPIOH);
	rcc_periph_clock_enable(RCC_GPIOI);
	rcc_periph_clock_enable(RCC_GPIOJ);
	rcc_periph_clock_enable(RCC_GPIOK);

    gpio_set(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_clear(GPIOK, GPIO7);
	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);

    gpio_setup();
    usart_setup();
    sdramsetup();

	usbd_dev = usbd_init(&stm32f207_usb_driver, &dev, &config,
		usb_strings, 3,
			usbd_control_buffer, sizeof(usbd_control_buffer));

//    lastusb();
	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
//    lastusb();
	nvic_enable_irq(NVIC_OTG_HS_IRQ);
    gpio_clear(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_set(GPIOK, GPIO7);
	while (1) {
    gpio_clear(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_set(GPIOK, GPIO7);
	
	ramtest();
cm_disable_interrupts();
		if (avaiable_incoming_endpoints)
		{
cm_enable_interrupts();
			uint8_t buf[MAX_USB_PACKET_SIZE];
			int len;
			for (i = 0; i < CDCACM_INTERFACE_COUNT; i ++)
				if (avaiable_incoming_endpoints & (1 << i))
				{
					len = incoming_usb_data[i].len;
					memcpy(buf, incoming_usb_data[i].buf, len);
cm_disable_interrupts();
					avaiable_incoming_endpoints ^= 1 << i;
					usbd_ep_nak_set(usbd_dev, incoming_usb_data[i].out_epnum, 0);
cm_enable_interrupts();
					if (len)
						while (usbd_ep_write_packet(usbd_dev, incoming_usb_data[i].in_epnum, buf, len) == 0xffff)
							busy_count ++;
					if (len == incoming_usb_data[i].max_packet_size)
					{
						while (usbd_ep_write_packet(usbd_dev, incoming_usb_data[i].in_epnum, 0, 0) == 0xffff)
							busy_count ++;
					}
				}
			continue;
		}
		// The 'wfi' instruction interferes with debugging, so keep it disabled for the time being.
		//__asm__("wfi");
cm_enable_interrupts();
	}
}


void otg_hs_isr()
{
	usbd_poll(usbd_dev);
}

