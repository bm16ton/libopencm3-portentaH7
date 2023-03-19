#!/bin/bash
sudo modprobe -r mtdblock
sudo modprobe -r spi-nor
sudo modprobe -r mcp251x
sudo rmmod spi_plat_usb
sudo rmmod spi_tiny_usb

sudo modprobe -r spi_plat_usb
sudo modprobe -r spi_tiny_usb
sudo rm /lib/modules/$(uname -r)/updates/dkms/spi_plat_usb.ko
sudo rm /lib/modules/$(uname -r)/updates/dkms/spi_tiny_usb.ko
sudo depmod
sudo cp mod1/*.ko /lib/modules/$(uname -r)/updates/dkms/
sudo cp mod2/*.ko /lib/modules/$(uname -r)/updates/dkms/
sudo depmod
