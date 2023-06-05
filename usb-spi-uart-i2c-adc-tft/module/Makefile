ifneq ($(KERNELRELEASE),)

obj-m := mod1/

else

KDIR ?= /lib/modules/`uname -r`/build

default:
	cd mod2 && $(MAKE)
	cp $(PWD)/mod2/Module.symvers $(PWD)/mod1/module.symvers
	cd mod1 && $(MAKE)
	cd mod3 && $(MAKE)
#	$(MAKE) -C $(KDIR) M=$(PWD)

endif

.PHONY: clean

clean:
	rm -f mod1/modules.order mod1/module.symvers mod1/Module.symvers mod1/*.o mod1/*.ko mod1/*.mod.* mod1/*.mod mod2/modules.order mod2/Module.symvers mod2/*.o mod2/*.ko mod2/*.mod.* mod2/*.mod modules.order Module.symvers *.o *.ko *.mod.* *.mod mod3/modules.order mod3/Module.symvers mod3/*.o mod3/*.ko mod3/*.mod.* mod3/*.mod

