#!/bin/bash
sudo modprobe -r mtdblock
sudo modprobe -r spi-nor
sudo modprobe -r mcp251x
sudo rmmod spi_plat_usb
sudo rmmod spi_tiny_usb
sudo rmmod usb_adc
sudo modprobe -r spi_plat_usb
sudo modprobe -r spi_tiny_usb
sudo modprobe -r usb_adc
sudo rm /lib/modules/$(uname -r)/updates/dkms/spi_plat_usb.ko
sudo rm /lib/modules/$(uname -r)/updates/dkms/spi_tiny_usb.ko
sudo rm /lib/modules/$(uname -r)/updates/dkms/usb_adc.ko
sudo depmod
sudo cp mod1/*.ko /lib/modules/$(uname -r)/updates/dkms/
sudo cp mod2/*.ko /lib/modules/$(uname -r)/updates/dkms/
sudo cp mod3/*.ko /lib/modules/$(uname -r)/updates/dkms/
sudo depmod
