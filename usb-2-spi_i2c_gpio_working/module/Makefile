
obj-m += spi-tiny-usb.o

ccflags-y := -Wno-declaration-after-statement

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

debug:
	KCPPFLAGS="-DDEBUG -std=gnu99"  make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
