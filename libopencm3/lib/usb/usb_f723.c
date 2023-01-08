/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2020 Stoyan Shopov <stoyan.shopov@gmail.com>
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

#include <string.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_hs.h>
#include <libopencm3/usb/dwc/dwc_otg.h>
#include "usb_private.h"
#include "usb_dwc_common.h"

#define REBASE(REG, ...)	REG(USB_OTG_HS_BASE, ##__VA_ARGS__)
//#define REBASE(REG, ...)	REG(host->backend->base_address, ##__VA_ARGS__)
/* Receive FIFO size in 32-bit words. */
#define RX_FIFO_SIZE 512

static usbd_device *stm32f723_usbd_init(void);

static struct _usbd_device usbd_dev;

const struct _usbd_driver stm32f723_usb_driver = {
	.init = stm32f723_usbd_init,
	.set_address = dwc_set_address,
	.ep_setup = dwc_ep_setup,
	.ep_reset = dwc_endpoints_reset,
	.ep_stall_set = dwc_ep_stall_set,
	.ep_stall_get = dwc_ep_stall_get,
	.ep_nak_set = dwc_ep_nak_set,
	.ep_write_packet = dwc_ep_write_packet,
	.ep_read_packet = dwc_ep_read_packet,
	.poll = dwc_poll,
	.disconnect = dwc_disconnect,
	.base_address = USB_OTG_HS_BASE,
	.set_address_before_status = 1,
	.rx_fifo_size = RX_FIFO_SIZE,
};



static void ulpi_pins(uint32_t gpioport, uint16_t gpiopins)
{
	gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, gpiopins);
	gpio_set_output_options(gpioport, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, gpiopins);
	gpio_set_af(gpioport, GPIO_AF10, gpiopins);
	
}
/** Initialize the USB device controller hardware of the STM32. */
/* Note: shopov - I compiled this initialization code from the libopencm3 usb_f207.c
 * source, and the st cube sources. Note that the st manuals state that some delays
 * are necessary at certain places. This code works for usb hosts that I tested on
 * without the delays, but this is bending the rules.
 *
 * If for someone the code below does not work, one thing to try is perhaps to
 * enable the delays before starting to debug other stuff. */
static usbd_device *stm32f723_usbd_init(void)
{
		rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
//	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14 | GPIO15);
//	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO14 | GPIO15);
//	gpio_set_af(GPIOB, GPIO_AF12, GPIO14 | GPIO15);
	ulpi_pins(GPIOA, GPIO3 | GPIO5);
	ulpi_pins(GPIOB, GPIO0 | GPIO1 | GPIO5  | GPIO10 | GPIO11 | GPIO12 | GPIO13);
	ulpi_pins(GPIOC, GPIO0 | GPIO2 | GPIO3);

	rcc_periph_clock_enable(RCC_OTGPHYC);
	rcc_periph_clock_enable(RCC_OTGHSULPI);
    rcc_periph_clock_enable(RCC_OTGHS);
	// TODO - check the preemption and subpriority, they are unified here
	nvic_set_priority(NVIC_OTG_HS_IRQ, 0);

	
//	OTG_HS_GINTSTS = OTG_GINTSTS_MMIS;

	// ??? What is this??? It is not documented in the manual
	OTG_HS_GCCFG |= OTG_GCCFG_PHYHSEN;

	/* ??? The st header files have this:
	 * #define USB_HS_PHYC_LDO_ENABLE                   USB_HS_PHYC_LDO_DISABLE
	 * ...go figure...
	 */
//	OTG_HS_PHYC_LDO |= OTG_PHYC_LDO_DISABLE;
//	while (!(OTG_HS_PHYC_LDO & OTG_PHYC_LDO_STATUS))
//		;
	/* This setting is for a HSE clock of 25 MHz. */
//	OTG_HS_PHYC_PLL1 = 5 << 1;
//	OTG_HS_PHYC_TUNE |= 0x00000F13U;
//	OTG_HS_PHYC_PLL1 |= OTG_PHYC_PLL1_ENABLE;
	/* 2ms Delay required to get internal phy clock stable */
	//HAL_Delay(2U);
		OTG_HS_GCCFG &= ~OTG_GCCFG_PWRDWN;


		/* Select External PHY */
		OTG_HS_GUSBCFG &= ~OTG_GUSBCFG_PHYSEL;
		
		
			REBASE(DWC_OTG_DCFG) = (REBASE(DWC_OTG_DCFG) & ~DWC_OTG_DCFG_DSPD_MASK) |
										DWC_OTG_DCFG_DSPD_FULL_2_0;
										
			//  LOOK HERE IF ISSUES
			REBASE(DWC_OTG_GUSBCFG) &= ~DWC_OTG_GUSBCFG_ULPIEVBUSD;
			
	//		OTG_HS_GCCFG |= OTG_GCCFG_PWRDWN;
			OTG_HS_GCCFG &= ~OTG_GCCFG_VBDEN;
			REBASE(DWC_OTG_GOTGCTL) |= DWC_OTG_GOTGCTL_BVALOEN |
											DWC_OTG_GOTGCTL_BVALOVAL;								
//	OTG_HS_GUSBCFG |= OTG_GUSBCFG_PHYSEL;
	/* Enable VBUS sensing in device mode and power down the PHY. */
	////OTG_HS_GCCFG |= OTG_GCCFG_VBUSBSEN | OTG_GCCFG_PWRDWN;

	/* Wait for AHB idle. */
	while (!(OTG_HS_GRSTCTL & OTG_GRSTCTL_AHBIDL));
	/* Do core soft reset. */
	REBASE(DWC_OTG_GRSTCTL) |= DWC_OTG_GRSTCTL_CSRST;
	while (REBASE(DWC_OTG_GRSTCTL) & DWC_OTG_GRSTCTL_CSRST);

	REBASE(DWC_OTG_DCTL) &= ~DWC_OTG_DCTL_SDIS;

	REBASE(DWC_OTG_GUSBCFG) |= DWC_OTG_GUSBCFG_FDMOD | DWC_OTG_GUSBCFG_TRDT_MASK;

	REBASE(DWC_OTG_GINTSTS) = DWC_OTG_GINTSTS_MMIS;

	/* Restart the PHY clock. */
	OTG_HS_PCGCCTL = 0;

	OTG_HS_GRXFSIZ = stm32f723_usb_driver.rx_fifo_size;
	usbd_dev.fifo_mem_top = stm32f723_usb_driver.rx_fifo_size;

	OTG_HS_DCTL |= OTG_DCTL_SDIS;
	//HAL_Delay(10);

	/* Unmask interrupts for TX and RX. */
	OTG_HS_GAHBCFG |= OTG_GAHBCFG_GINT;
	OTG_HS_GINTMSK = OTG_GINTMSK_ENUMDNEM |
			 OTG_GINTMSK_RXFLVLM |
			 OTG_GINTMSK_IEPINT |
			 OTG_GINTMSK_OEPINT |
			 //OTG_GINTMSK_USBRST |
			 OTG_GINTMSK_USBSUSPM |
			 OTG_GINTMSK_WUIM;

	OTG_HS_DAINTMSK = 0xffffffff;

	OTG_HS_DOEPMSK |= OTG_DOEPMSK_STUPM
		| OTG_DOEPMSK_XFRCM
		| OTG_DOEPMSK_EPDM
		| (1 << 5) //OTG_DOEPMSK_OTEPSPRM
		| (1 << 13) //OTG_DOEPMSK_NAKM
		;
	OTG_HS_DIEPMSK |= OTG_DIEPMSK_TOM
		| OTG_DIEPMSK_XFRCM
		| OTG_DIEPMSK_EPDM
		;

	OTG_HS_DCTL &=~ OTG_DCTL_SDIS;

	return &usbd_dev;
}

