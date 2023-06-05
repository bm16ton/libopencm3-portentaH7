ifneq ($(KERNELRELEASE),)
obj-m := spi_plat_usb.o

else


KDIR ?= /lib/modules/`uname -r`/build
export KBUILD_EXTRA_SYMBOLS=$(PWD)/module.symvers

default:
	$(MAKE) -C $(KDIR) M=$(PWD)

endif

.PHONY: clean

clean:
	rm -f modules.order Module.symvers *.o *.ko *.mod.* *.mod

