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
//#include "st7789_stm32_spi.h"
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "cdc.h"
#include "usb.h"
#include "bulkspi.h"

#include <libopencm3/stm32/syscfg.h>

#define USART_CONSOLE USART1
volatile uint32_t systick = 0;

int irqfire = 0;

void spiRelInit(void);
void spiInit(void);
void spi_test(void);

void get_buffered_line(void);
#define CDCACM_PACKET_SIZE 512
#define CDCACM_INTERFACE_0_DATA_IN_ENDPOINT 0x81
#define CDCACM_INTERFACE_0_DATA_OUT_ENDPOINT 1
#define CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT 0x82
#define CDCACM_INTERFACE_0_NOTIFICATION_IN_ENDPOINT_SIZE 64

int irqtype = EXTI_TRIGGER_FALLING;

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
	.idProduct = 0xc633,
	.bcdDevice = 0x0200,
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
		.bEndpointAddress = 0x02,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = BULK_EP_MAXPACKET,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x82,
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
		.bNumEndpoints = 2,
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
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char * usb_strings[] = {
	"16ton productions",
	"Envie",
	"TEST",
	"github.com/bm16ton",
	"usb-2-spi",
};

uint32_t SystemCoreClock = 400000;
void SysTick_IRQn_handler( void ) {
  ++systick;
}

static inline __attribute__((always_inline)) void __WFI(void)
{
	__asm volatile ("wfi");
}

#define GPIO1_PORT   GPIOA      
#define GPIO1_PIN    GPIO8     
#define GPIO2_PORT   GPIOC
#define GPIO2_PIN    GPIO7
#define GPIO3_PORT   GPIOG      
#define GPIO3_PIN    GPIO7      
#define GPIO4_PORT   GPIOJ
#define GPIO4_PIN    GPIO10
#define GPIO5_PORT   GPIOK
#define GPIO5_PIN    GPIO1
#define GPIO6_PORT
#define GPIO6_PIN
#define GPIO7_PORT
#define GPIO7_PIN
#define GPIO8_PORT
#define GPIO8_PIN

#define IRQ_TYPE_NONE		0
#define IRQ_TYPE_EDGE_RISING	0x00000001
#define IRQ_TYPE_EDGE_FALLING	0x00000002
#define IRQ_TYPE_EDGE_BOTH	IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING
#define IRQ_TYPE_LEVEL_HIGH	0x00000004
#define IRQ_TYPE_LEVEL_LOW	0x00000008

void delay_ms( uint32_t ms ) {
  // Calculate the 'end of delay' tick value, then wait for it.
  uint32_t next = systick + ms;
  while ( systick < next ) { __WFI(); }
}

static void my_delay_2( void )
{
	for (unsigned i = 0; i < 20000; i++)
	  {
		__asm__("nop");
	  }
}

static void my_delay_1( void )
{
   for (unsigned i = 0; i < 800000; i++)
     {
        __asm__("nop");
     }
}

static void irq_pin_init(void)
{
//    nvic_disable_irq(NVIC_EXTI0_IRQ);
    gpio_toggle(GPIOK, GPIO6);
	my_delay_2();
    nvic_enable_irq(NVIC_EXTI1_IRQ);					
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	my_delay_2();
	exti_select_source(EXTI1, GPIO5_PORT);
    exti_set_trigger(EXTI1, irqtype);
	exti_enable_request(EXTI1);
	irqfire = 1;
}

static void irq_none(void)
{
    gpio_toggle(GPIOK, GPIO5);
    nvic_disable_irq(NVIC_EXTI1_IRQ);
	my_delay_2();				
	exti_disable_request(EXTI1);
	my_delay_2();
	irqfire = 0;
}

static uint64_t sys_tick_counter;

uint32_t SysTick_Config(uint32_t ticks) {
(void)ticks;
systick_set_reload(400000000-1);

  STK_CSR  = STK_CSR_CLKSOURCE |
                   STK_CSR_TICKINT   |
                   STK_CSR_ENABLE;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
}

void sys_tick_handler(void)
{
	sys_tick_counter += 1000;
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

static void usbgpio_output(int gpio)
{
	if (gpio == 1) {
	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1_PIN);
	gpio_set_output_options(GPIO1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
							GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
    } else if (gpio == 2) {
	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	gpio_set_output_options(GPIO2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_set_output_options(GPIO3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set_output_options(GPIO4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	gpio_set_output_options(GPIO5_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}
	
    my_delay_1();
}

static void usbgpio_input(int gpio)
{

	if (gpio == 1) {
//	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1_PIN);
//	gpio_set(GPIO1_PORT, GPIO1_PIN);
    ;
	} else if (gpio == 2) {
	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}

	my_delay_1();
}

// When starting to learn coding I finally figured out how to send one number, but not receive so made
// usb to led drivers etc, then filled in the rest of control packet over time and finally rec. But looking
// at the following code you can see where and when I figured out the rest IE wValue, index,request

static enum usbd_request_return_codes usb_control_gpio_request(
    usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *, struct usb_setup_data *req))
{
    (void)complete;
	(void)usbd_dev;

    if (req->bRequest > 0x59) {
        return USBD_REQ_NEXT_CALLBACK;
    }
    
   (*len) = 1;
   (*buf)[0] = 1; //success

   if (req->wValue == 1)
     {
        if ( req->wIndex == 0 )
			{
				usbgpio_input(1);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				usbgpio_input(2);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 2 )
			{
				usbgpio_input(3);
				return USBD_REQ_HANDLED;
			}	
	    else if ( req->wIndex == 3 )
			{
				usbgpio_input(4);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				usbgpio_input(5);
				return USBD_REQ_HANDLED;
			}	
      }
   else if (req->wValue == 2)
     {
     if ( req->wIndex == 0 )
			{
				usbgpio_output(1);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				usbgpio_output(2);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 2 )
			{
				usbgpio_output(3);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 3 )
			{
				usbgpio_output(4);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				usbgpio_output(5);
				return USBD_REQ_HANDLED;
			}
      }
   else if (req->wValue == 3)
     {
     if ( req->wIndex == 0 )
			{
            if (gpio_get(GPIO1_PORT, GPIO1_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
            if (gpio_get(GPIO2_PORT, GPIO2_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
		    return USBD_REQ_HANDLED;
		   }
	    else if ( req->wIndex == 2 )
			{
            if (gpio_get(GPIO3_PORT, GPIO3_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
		 else if ( req->wIndex == 3 )
			{
            if (gpio_get(GPIO4_PORT, GPIO4_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
            if (gpio_get(GPIO5_PORT, GPIO5_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = 4;
			    return 1;
			} else {
				(*buf)[0] = 0; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = 4;
			    return 1;
			}
			return USBD_REQ_HANDLED;
			}
         }
  else if (req->wValue == 0)
     { 
	 if (req->bRequest == 1)
        {
        if ( req->wIndex == 0 )
			{
				gpio_set(GPIO1_PORT, GPIO1_PIN);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				gpio_set(GPIO2_PORT, GPIO2_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 2 )
			{
				gpio_set(GPIO3_PORT, GPIO3_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 3 )
			{
				gpio_set(GPIO4_PORT, GPIO4_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				gpio_set(GPIO5_PORT, GPIO5_PIN);
				return USBD_REQ_HANDLED;
			}
        }
   else if (req->bRequest == 0)
     {
     if (req->wIndex == 0)
			{
				gpio_clear(GPIO1_PORT, GPIO1_PIN);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				gpio_clear(GPIO2_PORT, GPIO2_PIN);
				return USBD_REQ_HANDLED;
			}	
		else if ( req->wIndex == 2 )
			{
				gpio_clear(GPIO3_PORT, GPIO3_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 3 )
			{
				gpio_clear(GPIO4_PORT, GPIO4_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				gpio_clear(GPIO5_PORT, GPIO5_PIN);
				return USBD_REQ_HANDLED;
			}	
	  }
    }
    else if (req->wValue == 9)
     {
     gpio_toggle(GPIOK, GPIO7);
     if ( req->wIndex == 1 ) {
        irqtype = IRQ_TYPE_NONE;
        irq_none();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 2 ) {
        irqtype = IRQ_TYPE_LEVEL_HIGH;
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 3 ) {
        irqtype = IRQ_TYPE_LEVEL_LOW;
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 4 ) {
//        irqtype = IRQ_TYPE_EDGE_BOTH;
        irqtype = EXTI_TRIGGER_BOTH;   
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 5 ) {
//        irqtype = IRQ_TYPE_EDGE_RISING;
        irqtype = EXTI_TRIGGER_RISING;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 6 ) {
//        irqtype = IRQ_TYPE_EDGE_FALLING;
        irqtype = EXTI_TRIGGER_FALLING;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     }
   else {
   
   return USBD_REQ_NEXT_CALLBACK;
   
   }
   
   return USBD_REQ_NEXT_CALLBACK;
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
static void usbgpio_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 9, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR,
				USB_REQ_TYPE_TYPE,
				usb_control_gpio_request);
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

    gpio_mode_setup(GPIOJ, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
	gpio_set_output_options(GPIOJ, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO11);
	
	gpio_set(GPIOJ, GPIO11);
//    gpio_clear(GPIOJ, GPIO11);
    
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
        .divq = 5U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
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

    sdramsetup();
    put_status("after sdram setup :");
	spiRelInit();
	put_status("after spiRelInit setuo :");
	// SPI register initialization
	spiInit();
//    gpio_clear(GPIOJ, GPIO11);
//	st_init();
	put_status("after stInit :");

	usbd_dev = usbd_init(&stm32f207_usb_driver, &dev, &config,
		usb_strings, 5,
			usbd_control_buffer, sizeof(usbd_control_buffer));


//	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
    usbd_register_set_config_callback(usbd_dev, usbgpio_set_config);
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
	irq_pin_init();
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
    usbd_ep_write_packet(usbd_dev, 0x81, buft, 4);
//    }
    exti_set_trigger(EXTI1, irqtype);
    printf("end of exti isr\r\n");
}


void otg_hs_isr()
{
	usbd_poll(usbd_dev);
}

