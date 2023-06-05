obj-m += usb_adc.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	.PHONY: clean

clean:
	rm -f modules.order Module.symvers *.o *.ko *.mod.* *.mod

