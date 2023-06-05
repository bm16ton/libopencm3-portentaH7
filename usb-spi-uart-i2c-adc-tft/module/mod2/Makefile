ifneq ($(KERNELRELEASE),)
obj-m := spi_tiny_usb.o


else

KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR)  M=$(PWD)
	cd ../
endif

.PHONY: clean

clean:
	rm -f modules.order Module.symvers *.o *.ko *.mod.* *.mod

