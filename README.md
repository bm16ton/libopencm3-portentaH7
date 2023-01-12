UPDATE: I2C now works, clock set/get added for i2c and qspi.

CURRENTLY WORKS: USBHS, i2c, external sdram, clock set and get for i2c/qspi.

Basic example of portenta h7 with libopencm3. IMPORTANT you must use the libopencm3 folder found in this repo and set device as stm32h747 with no trailing charactors. This example enumerates as usbcdc, initializes UART1 PA9/PA10 , inits the 8mb external ram. If performs basic read/write to ram and send results over USART1. The usbcdc in standard usbcdc example fashion sends back everything you type/send to it. 

For new projects, as mentioned use this repos libopencm3 and defone chip[ in your Makefile as "stm32h747". In the main .c file setup the ulpi gpio IE.


static void ulpi_pins(uint32_t gpioport, uint16_t gpiopins)
{
	gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, gpiopins);
	gpio_set_output_options(gpioport, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, gpiopins);
	gpio_set_af(gpioport, GPIO_AF10, gpiopins);
}

Then inside say main;


	ulpi_pins(GPIOA, GPIO3 | GPIO5);
	ulpi_pins(GPIOB, GPIO0 | GPIO1 | GPIO5  | GPIO10 | GPIO11 | GPIO12 | GPIO13);
	ulpi_pins(GPIOC, GPIO0);
	ulpi_pins(GPIOH, GPIO4);
	ulpi_pins(GPIOI, GPIO11); 
	
Amd init the f207 usb driver as usual.

TODO;

When I can get back to this I2C and SPI are of high priority. I see arduino "fixes up" 3.3v i2c controlleed regulator with sum register writes. Also the usb-c video out needs init to work (I believe the arduino bootloader does this automatically, but i want an implimentation in case we ditch that bootloader.

Add a usbcdc by default with set 1200baud to reset into arduino bootloader endpoint/callback


To upload from cli just use dfu-util like normal.


/usr/bin/dfu-util -a 0 --dfuse-address 0x08040000 -R -D firmware-name.bin
