#!/bin/bash
sudo modprobe -r mtdblock
sudo modprobe -r spi-nor
sudo rmmod spi_tiny_usb 


