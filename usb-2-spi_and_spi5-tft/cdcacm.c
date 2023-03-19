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
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "cdc.h"
#include "usb.h"
#include "bulkspi.h"
#include "usb-2-i2c/i2c_ctx.h"
#include <libopencm3/stm32/syscfg.h>
#include "tft_stm32_spi.h"

#define USART_CONSOLE USART1
volatile uint32_t systick = 0;



void spiRelInit(void);
void spiInit(void);
void spi_test(void);

void get_buffered_line(void);
#define CDCACM_PACKET_SIZE 512
#define CDCACM_INTERFACE_0_DATA_IN_ENDPOINT 0x81
#define CDCACM_INTERFACE_0_DATA_OUT_ENDPOINT 1
#define CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT 0x82
#define CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT_SIZE 64



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
	.iInterface = 0,

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
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0xC8,

	.interface = ifaces,
};

static const char * usb_strings[] = {
	"16ton productions",
	"Envie",
	"TEST",
	"github.com/bm16ton",
	"usb-2-spi",
};

void i2c_init(void);
uint32_t time_now(void);
uint64_t time64_now(void);

uint32_t SystemCoreClock = 400000;
void SysTick_IRQn_handler( void ) {
  ++systick;
}

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
	i2c_set_speed(I2C3, i2c_speed_fm_400k, 8);
	i2c_set_7bit_addr_mode(I2C3);
	i2c_peripheral_enable(I2C3);
	i2c_set_own_7bit_slave_address(I2C3, 0x00);

	for (uint32_t loop = 0; loop < 1500; ++loop) {
        __asm__("nop");
    }
	i2c_ctx_t ctx;
	i2c_ctx_init(&ctx, I2C3);
	i2c_ctx_reset(&ctx);

}

static uint64_t sys_tick_counter;

void sys_tick_handler(void)
{
	sys_tick_counter += 1000;
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
systick_set_reload(400000000-1);

  STK_CSR  = STK_CSR_CLKSOURCE |
                   STK_CSR_TICKINT   |
                   STK_CSR_ENABLE;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
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


bool rtc_init_flag_is_ready(void)
{
	return (RTC_ISR & RTC_ISR_INITF);
}

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding);


void usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{
printf("baud = %ld\r\n", coding->dwDTERate);
if (coding->dwDTERate == 1200) {
printf("entered reboot function \r\n");
rcc_periph_clock_enable(RCC_RTCAPB);
PWR_CR1 |= PWR_CR1_DBP;

RTC_WPR = 0XCA;
RTC_WPR = 0X53;
RTC_ISR |= RTC_ISR_INIT;
while (!rtc_init_flag_is_ready());
printf("done waiting for rtc init, now setting magicboot\r\n");  //todo used as delay cuz its almost perfect
__asm( "NOP" );
__asm( "NOP" );
__asm( "NOP" );
__asm( "NOP" );
RTC_BKPXR(0) = 0xDF59;  //arduino also has DFU_MAGIC_SERIAL_ONLY_RESET   0xb0  curious not in mcuboot code that i could see
__asm( "NOP" );
__asm( "NOP" );
__asm( "NOP" );
scb_reset_system();
}
//todo add usb to uart
/*
	usart_set_baudrate(USBUSART, coding->dwDTERate);

	if (coding->bParityType)
		usart_set_databits(USBUSART, (coding->bDataBits + 1 <= 8 ? 8 : 9));
	else
		usart_set_databits(USBUSART, (coding->bDataBits <= 8 ? 8 : 9));

	switch(coding->bCharFormat) {
	case 0:
		usart_set_stopbits(USBUSART, USART_STOPBITS_1);
		break;
	case 1:
		usart_set_stopbits(USBUSART, USART_STOPBITS_1_5);
		break;
	case 2:
	default:
		usart_set_stopbits(USBUSART, USART_STOPBITS_2);
		break;
	}

	switch(coding->bParityType) {
	case 0:
		usart_set_parity(USBUSART, USART_PARITY_NONE);
		break;
	case 1:
		usart_set_parity(USBUSART, USART_PARITY_ODD);
		break;
	case 2:
	default:
		usart_set_parity(USBUSART, USART_PARITY_EVEN);
		break;
	}
	*/
}



// TODO REALY NEED THAT USB-2-UART BEN!
/*
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
								 return USBD_REQ_HANDLED;
							 }
		case 0x21:
							 if (* len != 7)
								 return USBD_REQ_NOTSUPP;
							 memcpy(*buf, xbuf, 7);
							 return USBD_REQ_HANDLED;
		case USB_CDC_REQ_SET_LINE_CODING:
		    if (*len < sizeof(struct usb_cdc_line_coding))
			    return USBD_REQ_NOTSUPP;
		    usbuart_set_line_coding((struct usb_cdc_line_coding *)*buf);
		    return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}
*/
/*
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
int i;
int len;
uint8_t buf[CDCACM_PACKET_SIZE] = {0};

	len = usbd_ep_read_packet(usbd_dev, ep, buf, sizeof buf);
	if (len) {
		while (usbd_ep_write_packet(usbd_dev, 0x81, buf, len) == 0);
	}
}
*/


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
        .divq = 12U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 8U,    // MAY NEED TO BE 4 FOR SDRAM
    },
    .pll3 = {
        .divm = 5U,     // 2MHz PLL1 base clock pre-multiplier, cancels post div2.
        .divn = 160U,   // 5 x 160 = 800MHz
        .divp = 4U,     // 800 / 4 = 200
        .divq = 32U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
        .divr = 96U,                // 4mhz for i2c
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
    //  RCC_D2CCIP1R_SPI45SEL_APB4
    //  RCC_D2CCIP1R_SPI45SEL_PLL2Q
    rcc_set_spi45_clksel(RCC_D2CCIP1R_SPI45SEL_PLL3Q);
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

void printplls(void);

//silly functions I added to check all plls, seems like this probly exists in lopencm3 but I didnt see it
void printplls(void) {
printf("pll1p = %ld\r\n", rcc_get_pll1_clock('p'));
printf("pll1q = %ld\r\n", rcc_get_pll1_clock('q'));
printf("pll1r = %ld\r\n", rcc_get_pll1_clock('r'));
printf("pll2p = %ld\r\n", rcc_get_pll2_clock('p'));
printf("pll2q = %ld\r\n", rcc_get_pll2_clock('q'));
printf("pll2r = %ld\r\n", rcc_get_pll2_clock('r'));
printf("pll3p = %ld\r\n", rcc_get_pll3_clock('p'));
printf("pll3q = %ld\r\n", rcc_get_pll3_clock('q'));
printf("pll3r = %ld\r\n", rcc_get_pll3_clock('r'));
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



/*
void usbuart_send_stdout(const uint8_t *data, uint32_t len)
{
	while (len) {
		uint32_t cnt = CDCACM_PACKET_SIZE;
		if (cnt > len)
			cnt = len;
		nvic_disable_irq(USB_IRQ);
		cnt = usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT, data, cnt);
		nvic_enable_irq(USB_IRQ);
		data += cnt;
		len -= cnt;
		usbuart_set_led_state(TX_LED_ACT, true);
	}
}
*/

//THIS and a few other functions from above and in i2c files need to be broken out as external
//debug header

void put_status(char *m)
{
	uint16_t stmp;

	printf(m);
	printf(" Status: ");
	stmp = SPI_SR(SPI2);
	if (stmp & SPI_SR_TXC) {
		printf("TXC, TX COMPLETE BUS IDLE ");
	}
	if (stmp & SPI_SR_RXWNE) {
		printf("RXWNE, ");
	}
	if (stmp & SPI_SR_EOT) {
		printf("EOT, ");
	}
	if (stmp & SPI_SR_OVR) {
		printf("OVERRUN, REC FULL, ");
	}
	if (stmp & SPI_SR_MODF) {
		printf("MODE FAULT, ");
	}
	if (stmp & SPI_SR_CRCE) {
		printf("CRCE, ");
	}
	if (stmp & SPI_SR_UDR) {
		printf("UNDERRUN, ");
	}
	if (stmp & SPI_SR_RXP) {
		printf("COMPLETE DATA PKT READY FOR RX, ");
	}
	if (stmp & SPI_SR_TXP) {
		printf("WRITE FINISHED, ");
	}
	if (stmp & SPI_SR_DXP) {
		printf("BOTH TX/RX EVENTS PENDING, ");
	}
	if (stmp & SPI_SR_SUSP) {
		printf("MASTER SUSPENDED, ");
	}
	printf("\r\n");
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

//GOOD CANIDATE FOR THE DEBUG HEADER
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

  return 0; // lol
}

usbd_device *usbd_dev;
volatile uint32_t busy_count;
int main(void)
{
	SCB_VTOR = (uint32_t) 0x08040000;
    rcc_periph_clock_enable(RCC_GPIOH);
    rcc_periph_clock_enable(RCC_SYSCFG);
    rcc_periph_clock_enable(RCC_HSEM);   //MAINLY For booting M4
	volatile int i;
    InitLdoAndPll();
//    SCB_EnableICache();  //can configure and enable
//	SCB_EnableDCache();    //can enable no problem, but any config even known good working configs and everything locks
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


	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);
    setcm4bootadd0(0x08100000 >> 16);
//    bootboth();
//    forcem4boot();
    gpio_setup();
    gpio_set(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_clear(GPIOK, GPIO7);
    usart_setup();
    printf("m4boot0addr = 0x%04x\r\n", getcm4bootadd0());
    i2c_init();
    sdramsetup();
    put_status("after sdram setup :");
	spiRelInit();
	put_status("after spiRelInit setuo :");
	// SPI register initialization
	spiInit();
	st_init();
	put_status("after stInit :");

	usbd_dev = usbd_init(&stm32f207_usb_driver, &dev, &config,
		usb_strings, 5,
			usbd_control_buffer, sizeof(usbd_control_buffer));


//	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
    usbd_register_set_config_callback(usbd_dev, usb_set_config);
//    usbd_register_set_config_callback(usbd_dev, usbgpio_set_config);
    usbd_register_set_config_callback(usbd_dev, usbspi_set_config);

	nvic_enable_irq(NVIC_OTG_HS_IRQ);
    gpio_clear(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_set(GPIOK, GPIO7);
	ramtest();
	printplls();

	printf("spi2 clock freq  %ld \r\n", rcc_get_spi_clk_freq(SPI2));
    printf("qspi clock freq  %ld \r\n", rcc_get_qspi_clk_freq(RCC_QSPI));
	printf("fmc clock freq  %ld \r\n", rcc_get_fmc_clk_freq(RCC_FMC));
	printf("usart1 clock = %ld \r\n", rcc_get_usart_clk_freq(USART1));
	printf("cpu clock = %ld \r\n", rcc_get_bus_clk_freq(RCC_CPUCLK));
	printf("RCC_PERCLK = %ld \r\n", rcc_get_bus_clk_freq(RCC_PERCLK));
	printf("sysclock = %ld \r\n", rcc_get_bus_clk_freq(RCC_SYSCLK));
	printf("m4 core clock = %ld \r\n", rcc_get_core2_clk_freq());
	printf("I2C3 clock = %ld \r\n", rcc_get_i2c_clk_freq(I2C3));
	gpio_clear(GPIOK, GPIO5);
    gpio_set(GPIOK, GPIO6);
	gpio_set(GPIOK, GPIO7);
//	irq_pin_init();
	while (1) {

	;


	}
}

void exti1_isr(void)
{

    // char buf2[64] __attribute__ ((aligned(4)));
    printf("interrupt fired\r\n");
    uint8_t buft[4] = {3, 3, 3, 3};
	exti_reset_request(EXTI1);
//	usbd_ep_write_packet(usbd_device usbd_dev, 0x83, buf2, 64);
//if (irqfire == 1) {
    usbd_ep_write_packet(usbd_dev, 0x82, buft, 4);
//    }
    exti_set_trigger(EXTI1, irqtype);
    printf("end of exti isr\r\n");
}

void otg_hs_isr()
{
	usbd_poll(usbd_dev);
}

