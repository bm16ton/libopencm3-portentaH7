UPDATE: added basic yet largly functional usb-2-spi (well and usb-2-gpio/irq) with kernel driver. It needs major cleanup all unusedd code removed, untested config options tested, the usb-2-i2c-adc etc added. Some of these things adc/spi maybe a great excuse for me to finally play around with compression on mcu usb device and kernel driver. Notro does it with GUD on pi pico https://github.com/notro/gud-pico (seriously follow Notro guy always does the coolest shit and seriously knows how to code. Anyway I wannna play with the compression stuff. Shit to remind myself Ill write it here...Add spi flash protocol stuff the kernel supports. 

UPDATED LIST OF WHAT WORKS; USBHS, I2c, ADC, SPI, M4 BOOTING, EXTERNAL SDRAM, UART, RTC BACKUP REGISTERS/AUTOBOOT BOOTLOADER, CLOCK SET/GET I2C/QSPI/CRAP others I forgot. Seems like im forgetting something.

UPDATE: can now boot m4 core, added functions;
setcm4bootadd0
setcm4bootadd1
bootboth (needs new name)
forcem4boot
getcm4bootadd0

UPDATE: ADC works the one example to rule them all enumerates with bulk endpoints, use the linux kernel module in this repo, it should create /dev/adc-wmagic1 simply cat it like so "sudo cat /dev/adc-wmagic1" This will print human readable (slow) adc readings, or search for humanreadable in /sys IE "sudo find /sys -iname humanreadable" and switch to binary output IE "echo 0 | sudo tee -a /sys/devices/platform/soc@0/a8f8800.usb/a800000.usb/xhci-hcd.2.auto/usb3/3-1/3-1.1/3-1.1.2/3-1.1.2.2/3-1.1.2.2.3/3-1.1.2.2.3:1.2/humanreadable" then cat /dev/adc-wmagic1 with whatever hex converter/dump you like, IE "sudo cat /dev/adc-wmagic1 | hd" or ""sudo cat /dev/adc-wmagic1 | xxd" the list of possabilities is long.

UPLOAD SCRIPT; Just a basic upload script, does need the example from listSerialPortsC for autodetecting which usb-2-serial is the portenta. In firmware the timing on unlocking/writing the rtc backup registers seems to be important/specific, currently sum nops and a printf are serving that purpose and are close enuff to work 98% of the time. If it simply reboots without entering bootloader just run the upload script again and it should work, no idea why the timen is slightly diff on sum attempts keeping the registers from being written to. 

UPDATE: SPI now works tested on ili9241 (thanks to https://github.com/libopencm3/libopencm3/issues/1392), The example also now reboots to bootloader on baud rate change 1200, and auto reboots back. RTC backup registers can unlock/edit for bootloader. I think maybe another clock set/get was added but dont remember atm. QSPI flash should be working, think I need to figure out the last of the mapping stuff to test (need to be added to ld? seems likely, never used external flash before so need to readup). The dma headers copied into the stm32h7 lib folder are just for the bare basic register info to hopefully get a bare metal dma example working, doubtful if not impossible that some of the ones I dont require for such a task are totally incorrect.(So dont trust/use them without checking first).

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
